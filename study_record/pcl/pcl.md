# pcl

https://github.com/HuangCongQing/pcl-learning/blob/master/PCL%E5%AD%A6%E4%B9%A0%E6%8C%87%E5%8D%97%26%E8%B5%84%E6%96%99%E6%8E%A8%E8%8D%90.md

### OpenPCDet

- **GitHub 链接**：[GitHub - open-mmlab/OpenPCDet: OpenPCDet Toolbox for LiDAR-based 3D Object Detection.](https://github.com/open-mmlab/OpenPCDet)

- **功能简介**：
  
  专注于**点云 3D 目标检测**的开源框架（属于 OpenMMLab 生态），集成了多种 SOTA 算法（如 PointPillars、SECOND、PV-RCNN 等）。
  
  支持自动驾驶场景的主流数据集（KITTI、Waymo、nuScenes 等），提供完整的训练 pipeline（数据预处理、模型训练、评估、可视化），适合快速复现或改进 3D 目标检测模型，基于 PyTorch 实现。

## 下载anaconda

## 创建虚拟环境

```bash
conda create --prefix  ~/develop/dependence/pcl python=3.10
```

安装torch

......

## OpenPCDet 工具包是做什么的？

注：我们已将 PCDet 从 v0.1 版本升级至 v0.2 版本，采用全新架构，可支持多种数据集和模型。

OpenPCDet 是一个基于 PyTorch 的通用代码库，用于点云的 3D 目标检测。它目前支持多种最先进的 3D 目标检测方法，为单阶段和双阶段 3D 检测框架提供了高度重构的代码。

依托 OpenPCDet 工具包，我们在 Waymo 开放数据集挑战赛中，斩获了纯激光雷达方法类别下 3D 检测、3D 跟踪、领域自适应三个赛道的冠军。相关 Waymo 模型即将在 OpenPCDet 中发布。

该代码库目前正持续更新，后续将支持更多数据集和模型，也欢迎大家贡献代码。

### OpenPCDet 设计模式

采用数据 - 模型分离架构，搭配统一的点云坐标系统，可轻松扩展至自定义数据集。

![](/home/maple/笔记/images/2025-11-11-20-03-05-2025-11-11%2020-00-36屏幕截图.png)

> 这张图展示了**OpenPCDet 工具包的核心设计流程**，体现了其 **“数据 - 模型分离 + 统一坐标系”** 的架构理念，具体模块解释如下：
> 
> ### 1. 「Data（数据）」模块
> 
> - **数据集层**：支持 KITTI、Waymo、NuScenes 等多种主流 3D 检测数据集，所有数据集都通过 **“unified coordinate（统一坐标系）”** 进行格式对齐，确保空间语义的一致性。
> - **数据预处理层（Data Preparation）**：包含`augmentor`（数据增强，如点云旋转、缩放）和`processor`（数据处理，如点云下采样、特征编码），将原始数据处理为模型可输入的格式。
> 
> ### 2. 「Model（模型）」模块
> 
> - **检测前向传播（Detector.forward）**：模型的核心推理环节，同时支撑 ** 训练（train）**和**测试（Test）** 流程。
>   - 训练时，连接到 **Optimization（优化）** 模块，完成损失计算、参数更新等训练逻辑。
>   - 测试时，进入 ** 后处理（Post-processing）** 模块，通过`nms`（非极大值抑制）和`score thresh`（分数阈值过滤）筛选出最终检测框。
> 
> ### 3. 「结果闭环与评估」
> 
> 模型输出的`prediction（预测结果）`会通过**统一坐标系**反向映射到数据层，生成`Prediction-dicts`（预测字典，包含检测框、置信度等信息），最终进入 **Evaluation（评估）** 模块，完成模型性能的量化分析（如 mAP、召回率等指标）。
> 
> ### 4. 「统一坐标系」的作用
> 
> 右下角的`Unified normative coordinate`定义了统一的空间坐标系（x、y、z 轴规范），是 “数据 - 模型分离” 设计的核心保障 —— 无论数据集原始坐标系如何，都先转换到该统一坐标系下处理，确保模型在一致的空间语义中推理，同时也让新增自定义数据集 / 模型的扩展更便捷。
> 
> 简言之，这张图完整呈现了 OpenPCDet 从 **“数据输入→预处理→模型训练 / 测试→结果后处理→性能评估”** 的全流程，清晰体现了其模块化、易扩展的架构优势。

**统一的 3D 边界框定义：(x, y, z, dx, dy, dz, 朝向角)。**

**灵活清晰的模型结构，以轻松支持各种 3D 检测模型。**

![](/home/maple/笔记/images/2025-11-11-20-11-42-2025-11-11%2020-10-59屏幕截图.png)

> 这张图展示了**OpenPCDet 中 3D 目标检测模型的典型架构流程**，从点云数据输入到预测结果输出，分为**Backbone3D、Backbone2D、DenseHead、RoIHead**四大核心模块，以下是各模块的详细解释：
> 
> ### 1. Backbone3D（3D 特征提取骨干网络）
> 
> 负责从原始点云提取 3D 特征，包含两条并行的特征提取支路：
> 
> - **支路 1（基于稀疏卷积）**：
>   - `VFE`（Voxel Feature Encoding，体素特征编码）：将点云划分为体素，对每个体素内的点云特征进行编码。
>   - `3D SparseConv`（3D 稀疏卷积）：对体素特征进行 3D 稀疏卷积操作，提取三维空间中的特征表示。
> - **支路 2（基于 PointNet++）**：
>   - `PointNet++`：直接对原始点云进行分层特征提取（无需体素化），捕捉点云的局部和全局特征。
>   - `Point Feature Encoding`：对 PointNet++ 提取的点特征进行编码，与稀疏卷积支路的特征进行融合或适配。
> 
> ### 2. Backbone2D（2D 特征提取骨干网络）
> 
> 将 3D 特征映射到鸟瞰图（BEV）并提取 2D 特征：
> 
> - `Map_to_BEV`：将 3D 稀疏卷积的输出转换为 ** 鸟瞰图（Bird's-Eye View）** 表示，把三维空间特征投影到二维平面。
> - `Encoder conv2d`：对 BEV 特征图进行 2D 卷积编码，提取更抽象的二维特征表示。
> 
> ### 3. DenseHead（密集预测头）
> 
> 基于 2D 和 3D 特征进行密集检测（单阶段检测逻辑）：
> 
> - `RPN Head`（Region Proposal Network Head）：基于 BEV 的 2D 特征，生成初始的 3D 候选框（proposal），预测目标的位置、尺寸和类别。
> - `Point Head`：基于 PointNet++ 提取的点特征，直接对原始点云进行逐点或局部的目标预测，补充细粒度的特征信息。
> 
> ### 4. RoIHead（感兴趣区域头，两阶段检测逻辑）
> 
> 对候选框进行精细化优化：
> 
> - `Proposal Layer`：从 RPN Head 的输出中筛选出高质量的候选框。
> - `RoI feature extraction`：针对每个候选框，提取其对应的区域特征（可结合 3D 和 2D 特征）。
> - `RoI Head`：对候选框的位置、尺寸、类别进行二次优化，输出最终的检测结果。
> 
> ### 整体流程总结
> 
> 原始`Point Cloud Data`（点云数据）输入后，先由`Backbone3D`提取 3D 特征，再经`Backbone2D`转换为 BEV 的 2D 特征；随后`DenseHead`进行密集预测生成候选框，`RoIHead`对候选框精细化处理，最终输出`Prediction Results`（检测结果）。
> 
> 该架构同时支持 ** 单阶段（DenseHead 直接输出）**和**两阶段（RoIHead 优化）** 的 3D 目标检测流程，通过多支路特征融合（体素 + 点云 + BEV），兼顾了检测的速度与精度。

**在同一个框架内支持多种模型，例如**

![](/home/maple/笔记/images/2025-11-11-20-20-53-2025-11-11%2020-20-42屏幕截图.png)

> 这张图展示了 **OpenPCDet 框架中支持的多种 3D 目标检测模型的结构对比**，通过模块化拆分（Backbone3D、Backbone2D、DenseHead、RoIHead）直观呈现不同模型的设计差异。以下是对每个模型组的详细解释：
> 
> ### 1. VoxelNet / SECOND / VoxelFPN
> 
> - **Backbone3D**：采用 “VFE（体素特征编码）+ 3D 稀疏卷积” 提取体素级 3D 特征，同时结合 PointNet++ 进行点云特征编码。
> - **Backbone2D**：将 3D 特征 “Reshape 到 BEV（鸟瞰图）”，再通过 2D 卷积编码器提取平面特征。
> - **DenseHead**：通过 RPN Head 生成候选框，Point Head 补充点云细粒度特征。
> - **RoIHead**：对候选框进行 “特征提取 + 精细化优化”，输出最终检测结果。
> 
> ### 2. PointPillar / PIXOR / CenterPoint
> 
> - **Backbone3D**：VFE 处理后直接 “Pass” 到 2D 模块（无 3D 稀疏卷积），PointNet++ 负责点云特征编码。
> - **Backbone2D**：采用 “PillarScatter” 将点云柱特征映射到 BEV，再通过 2D 卷积编码器处理。
> - **DenseHead & RoIHead**：与其他模型结构类似，通过 RPN Head 生成候选框，RoIHead 完成精细化优化。
> 
> ### 3. VoteNet / 3DSSD
> 
> - **Backbone3D**：VFE + 3D 稀疏卷积提取 3D 特征，PointNet++ 辅助点云特征编码。
> - **Backbone2D**：将 3D 特征 “Map 到 BEV” 后，通过 2D 卷积编码器处理。
> - **DenseHead & RoIHead**：延续 “候选框生成 + 精细化优化” 的两阶段逻辑。
> 
> ### 4. PointRCNN / STD
> 
> - **Backbone3D**：VFE + 3D 稀疏卷积提取体素特征，PointNet++ 模块 “Pass”（不参与特征编码）。
> - **Backbone2D**：3D 特征 “Map 到 BEV” 后经 2D 卷积编码。
> - **RoIHead**：采用 “Point Cloud Region Pooling” 对候选框区域的点云特征进行聚合，再通过 PointRCNN Head 优化。
> 
> ### 5. PartA2-Net
> 
> - **Backbone3D**：VFE + 3D 稀疏卷积提取体素特征，PointNet++ 模块 “Pass”。
> - **Backbone2D**：3D 特征 “Reshape 到 BEV” 后经 2D 卷积编码。
> - **RoIHead**：通过 “RoI-wise Pooling” 对候选框区域特征进行池化，再由 PartA2 Head 完成精细化检测。
> 
> ### 6. PV-RCNN
> 
> - **Backbone3D**：VFE + 3D 稀疏卷积提取体素特征，结合 PointNet++ 的 “VSA（体素集抽象）” 模块融合点云全局特征。
> - **Backbone2D**：3D 特征 “Reshape 到 BEV” 后经 2D 卷积编码。
> - **RoIHead**：采用 “RoI-grid Pooling” 对候选框区域进行网格级特征聚合，由 PVRCNN Head 输出最终结果。
> 
> ### 7. Voxel R-CNN
> 
> - **Backbone3D**：VFE + 3D 稀疏卷积提取体素特征，PointNet++ 模块 “Pass”。
> - **Backbone2D**：3D 特征 “Reshape 到 BEV” 后经 2D 卷积编码。
> - **RoIHead**：通过 “Voxel RoI Pooling” 对候选框区域的体素特征进行池化，再由 RoI Head 完成优化。
> 
> ### 总结
> 
> 这张图通过**模块化对比**，清晰展现了 OpenPCDet 对不同 3D 检测模型的支持逻辑：
> 
> - 体素化模型（如 VoxelNet、SECOND）依赖 “3D 稀疏卷积 + BEV 特征”；
> - 点云直接处理模型（如 PointRCNN、PV-RCNN）侧重 “点云特征聚合 + 区域精细化”；
> - 支柱类模型（如 PointPillar）则通过 “PillarScatter” 实现高效的 BEV 特征转换。
> 
> 这种模块化设计体现了 OpenPCDet 的**灵活性**—— 不同模型可在同一框架下共享数据处理和评估流程，便于研究人员对比、复现和改进各类 3D 检测算法。

**当前支持的特性**

- **支持单阶段和双阶段 3D 目标检测框架**
  - 说明：OpenPCDet 工具包能够兼容和实现这两种主流的 3D 检测算法架构。
- **支持多 GPU 和多机器的分布式训练与测试**
  - 说明：通过利用多个 GPU 或多台计算机的计算资源，可以显著提升模型训练和评估的速度。
- **支持在不同尺度上使用多个检测头（Heads）来检测不同类别**
  - 说明：该框架允许模型针对不同大小的物体（例如，小汽车和行人）或不同类别的物体，使用专门的 “检测头” 进行更精准的识别。
- **支持堆叠式体素集抽象（Stacked Voxel Set Abstraction），以编码不同场景中的不同数量的点**
  - 说明：这是一种处理点云数据的高级技术，能够有效地从稀疏且数量变化的点云中提取特征，适应不同复杂程度的场景。
- **支持用于目标分配的自适应训练样本选择（ATSS）**
  - 说明：在训练过程中，该算法可以智能地选择哪些样本对模型学习最有帮助，从而提高训练效率和模型性能。
- **支持 RoI-aware 点云池化和 RoI-grid 点云池化**
  - 说明：这些是两种精细的特征提取方法，能够在感兴趣区域（RoI）内聚合点云信息，为后续的目标分类和定位提供更精确的特征。
- **支持 GPU 加速版的 3D IoU 计算和旋转 NMS（非极大值抑制）**
  - 说明：这是两个关键的后处理步骤。
  - **3D IoU (Intersection over Union)**：用于计算两个 3D 边界框的重叠程度，是评估检测精度的核心指标。
  - **旋转 NMS (Rotated Non-Maximum Suppression)**：用于从多个候选检测框中筛选出最佳结果，尤其适用于处理具有旋转角度的目标。
  - GPU 加速意味着这些计算都在显卡上进行，速度非常快

### KITTI 3D 目标检测基准模型

下表列出了部分支持的方法，结果为 KITTI 数据集验证集上中等难度的 3D 检测性能。

所有基于激光雷达的模型均使用 8 块 GTX 1080Ti GPU 训练，且可提供下载。

训练时间基于 8 块 TITAN XP GPU 和 PyTorch 1.5 版本测得。

要不要我帮你整理一份**KITTI 基准模型训练配置与性能关键信息对照表**？
