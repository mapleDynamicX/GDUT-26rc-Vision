# train

## pcdtobin

```python
# -*- coding: utf-8 -*-
# @Time : 2022/7/25 11:30
# @Author : JulyLi
# @File : pcd2bin.py

# 导入 numpy 库，用于进行数值计算和数组操作
import numpy as np
# 导入 os 库，用于与操作系统交互，如文件路径处理、目录创建等
import os
# 导入 argparse 库，用于解析命令行参数
import argparse
# 从 pypcd 库中导入 pypcd 模块，用于读取和处理 PCD (Point Cloud Data) 文件
from pypcd import pypcd
# 导入 csv 库，用于读写 CSV (Comma-Separated Values) 文件
import csv
# 从 tqdm 库中导入 tqdm 函数，用于在循环中显示进度条
from tqdm import tqdm

# 定义主函数，程序的入口点
def main():
    ## Add parser
    # 创建一个参数解析器对象，并添加描述信息
    parser = argparse.ArgumentParser(description="Convert .pcd to .bin")
    # 为解析器添加一个名为 "--pcd_path" 的参数
    parser.add_argument(
        "--pcd_path",
        help=".pcd file path.",  # 参数的帮助说明
        type=str,               # 参数的数据类型
        default="pcd_raw1"      # 参数的默认值
    )
    # 为解析器添加一个名为 "--bin_path" 的参数
    parser.add_argument(
        "--bin_path",
        help=".bin file path.",
        type=str,
        default="bin"
    )
    # 为解析器添加一个名为 "--file_name" 的参数
    parser.add_argument(
        "--file_name",
        help="File name.",
        type=str,
        default="file_name"
    )
    # 解析命令行参数，并将结果存储在 args 对象中
    args = parser.parse_args()

    ## Find all pcd files
    # 初始化一个空列表，用于存储找到的 PCD 文件路径
    pcd_files = []
    # 遍历指定目录（args.pcd_path）及其所有子目录
    for (path, dir, files) in os.walk(args.pcd_path):
        # 遍历当前目录下的所有文件
        for filename in files:
            # 打印文件名（此行为注释掉的调试信息）
            # print(filename)
            # 分离文件名和文件扩展名
            ext = os.path.splitext(filename)[-1]
            # 如果文件扩展名是 .pcd，则将其完整路径添加到列表中
            if ext == '.pcd':
                pcd_files.append(path + "/" + filename)

    ## Sort pcd files by file name
    # 对找到的 PCD 文件路径列表按文件名进行排序
    pcd_files.sort()
    # 打印提示信息，告知用户已完成点云文件的加载
    print("Finish to load point clouds!")

    ## Make bin_path directory
    try:
        # 检查输出目录（args.bin_path）是否存在
        if not (os.path.isdir(args.bin_path)):
            # 如果不存在，则创建该目录
            os.makedirs(os.path.join(args.bin_path))
    except OSError as e:
        # 如果创建目录时发生错误（如权限问题），则捕获异常并重新抛出
        # if e.errno != errno.EEXIST:
        #     print("Failed to create directory!!!!!")
            raise

    ## Generate csv meta file
    # 定义 CSV 元数据文件的完整路径
    csv_file_path = os.path.join(args.bin_path, "meta.csv")
    # 以写入模式（'w'）打开 CSV 文件
    csv_file = open(csv_file_path, "w")
    # 创建一个 CSV 写入器对象，并指定分隔符、引用符等格式
    meta_file = csv.writer(
        csv_file, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL
    )
    ## Write csv meta file header
    # 向 CSV 文件中写入表头行
    meta_file.writerow(
        [
            "pcd file name",  # PCD 文件名
            "bin file name",  # 对应的 BIN 文件名
        ]
    )
    # 打印提示信息，告知用户已完成 CSV 元数据文件的创建
    print("Finish to generate csv meta file")

    ## Converting Process
    # 打印提示信息，告知用户转换过程即将开始
    print("Converting Start!")
    # 初始化一个序列计数器，用于生成连续的 BIN 文件名
    seq = 0
    # 使用 tqdm 包装文件列表，以便在循环时显示进度条
    for pcd_file in tqdm(pcd_files):
        ## Get pcd file
        # 从指定的 PCD 文件路径加载点云数据
        pc = pypcd.PointCloud.from_path(pcd_file)

        ## Generate bin file name
        # 生成 BIN 文件的文件名（被注释掉的是另一种命名方式）
        # bin_file_name = "{}_{:05d}.bin".format(args.file_name, seq)
        bin_file_name = "{:05d}.bin".format(seq)  # 格式化为 5 位数字的文件名
        # 拼接 BIN 文件的完整输出路径
        bin_file_path = os.path.join(args.bin_path, bin_file_name)

        ## Get data from pcd (x, y, z, intensity, ring, time)
        # 从点云数据中提取 'x' 坐标，并转换为 numpy 数组和 float32 类型
        np_x = (np.array(pc.pc_data['x'], dtype=np.float32)).astype(np.float32)
        # 提取 'y' 坐标
        np_y = (np.array(pc.pc_data['y'], dtype=np.float32)).astype(np.float32)
        # 提取 'z' 坐标
        np_z = (np.array(pc.pc_data['z'], dtype=np.float32)).astype(np.float32)
        # 提取 'intensity'（强度）值，并归一化到 0-1 范围
        np_i = (np.array(pc.pc_data['intensity'], dtype=np.float32)).astype(np.float32) / 256
        # 提取 'ring'（环号）值（被注释掉，未使用）
        # np_r = (np.array(pc.pc_data['ring'], dtype=np.float32)).astype(np.float32)
        # 提取 'time'（时间戳）值（被注释掉，未使用）
        # np_t = (np.array(pc.pc_data['time'], dtype=np.float32)).astype(np.float32)

        ## Stack all data
        # 将 x, y, z, i 四个数组垂直堆叠，然后转置为 (N, 4) 的形状（N 为点的数量）
        points_32 = np.transpose(np.vstack((np_x, np_y, np_z, np_i)))

        ## Save bin file
        # 将处理后的点云数据以二进制格式写入到指定的 BIN 文件中
        points_32.tofile(bin_file_path)

        ## Write csv meta file
        # 向 CSV 元数据文件中写入一行，记录当前 PCD 文件及其对应的 BIN 文件名
        meta_file.writerow(
            [os.path.split(pcd_file)[-1], bin_file_name]
        )

        # 序列计数器自增 1，为下一个文件做准备
        seq = seq + 1

# 如果当前脚本是作为主程序运行（而不是被其他脚本导入），则调用 main() 函数
if __name__ == "__main__":
    main()
```

## custom_dataset.yaml

```yaml
# -----------------------------------------------------------------------------
# 1. 数据集基础信息配置
# -----------------------------------------------------------------------------

# 指定使用的数据集类。这里使用的是用户自定义的数据集类 'CustomDataset'。
# OpenPCDet会通过这个类来加载和解析你的数据。
DATASET: 'CustomDataset'

# 数据集的根路径。所有数据文件（如点云、标注文件）都相对于这个路径查找。
DATA_PATH: '../data/custom'

# 一个重要的注释：如果修改了本配置文件中的某些关键结构（例如新增了数据增强或处理器），
# 可能需要同步修改 pcdet/models/detectors/detector3d_template.py 文件中的
# Detector3DTemplate::build_networks 函数里的 model_info_dict，以确保模型能正确识别新的输入信息。
# 通常这一步在较新版本的OpenPCDet中会自动处理，但作为最佳实践，需要留意。

# 定义点云的有效坐标范围。格式：[x_min, y_min, z_min, x_max, y_max, z_max]
# 所有超出此范围的点将在数据预处理阶段被过滤掉，以减少计算量和噪声。
POINT_CLOUD_RANGE: [-70.4, -40, -3, 70.4, 40, 1] # x=[-70.4, 70.4], y=[-40,40], z=[-3,1]

# -----------------------------------------------------------------------------
# 2. 数据分割与元信息配置
# -----------------------------------------------------------------------------

# 定义训练集和测试集的划分方式。
# 'train' 对应的数据子集名称为 'train'
# 'test' 对应的数据子集名称为 'val' (这里用验证集来测试，是常见做法)
DATA_SPLIT: {
    'train': train,
    'test': val
}

# 指定训练集和测试集的信息文件（.pkl）路径。
# 这些.pkl文件通常包含了数据集的详细元信息，例如每个样本的点云文件路径、
# 标注框（gt_boxes）、类别等。
INFO_PATH: {
    'train': [custom_infos_train.pkl],
    'test': [custom_infos_val.pkl],
}

# 指定从数据样本中需要加载的字段列表。
# 这里只加载了 'points'（点云数据本身）。
# 根据需要，还可以加载 'calib' (相机-激光雷达标定参数), 'gt_boxes' 等。
GET_ITEM_LIST: ["points"]

# 是否只保留在相机视野（Field of View, FOV）内的点。
# 设置为True可以过滤掉视野外的无效点，提高效率。
FOV_POINTS_ONLY: True

# -----------------------------------------------------------------------------
# 3. 点云特征编码配置
# -----------------------------------------------------------------------------

# 配置点云特征的编码方式。
POINT_FEATURE_ENCODING:
    # 特征编码类型。'absolute_coordinates_encoding' 表示直接使用点的绝对坐标作为特征。
    # 其他可选类型如 'relative_coordinates_encoding'（相对某个中心点的坐标）。
    encoding_type: absolute_coordinates_encoding,

    # 模型实际使用的特征列表。这里使用了 x, y, z 坐标和反射强度 intensity。
    used_feature_list: ['x', 'y', 'z', 'intensity'],

    # 原始点云中对应的特征列表。需要与 used_feature_list 一一对应。
    # 例如，如果原始数据中强度字段名为 'i'，这里就需要改为 'i'。
    src_feature_list: ['x', 'y', 'z', 'intensity'],

# -----------------------------------------------------------------------------
# 4. 数据增强配置 (DATA_AUGMENTOR)
# 这些配置用于在训练过程中对数据进行随机变换，以增强模型的泛化能力。
# 此处配置与PV-RCNN模型的配置相同。
# -----------------------------------------------------------------------------

DATA_AUGMENTOR:
    # 指定需要禁用的数据增强方法列表。'placeholder' 是一个占位符，表示无禁用项。
    DISABLE_AUG_LIST: ['placeholder']

    # 数据增强方法的详细配置列表。
    AUG_CONFIG_LIST:
        # 4.1. 从数据库中采样额外的目标（GT Sampling）
        # 该方法从一个预先准备好的目标数据库中随机采样一些目标，并将它们放入当前场景中，
        # 以解决某些类别人数过少（类别不平衡）的问题。
        - NAME: gt_sampling
          # 是否使用路面平面信息来辅助采样。设置为False表示不使用。
          USE_ROAD_PLANE: False

          # 目标数据库信息文件的路径。该文件记录了数据库中每个目标的点云、类别、包围框等信息。
          DB_INFO_PATH:
              - custom_dbinfos_train.pkl # 参考: pcdet/datasets/augmentor/database_ampler.py:line 26

          # 从数据库中读取目标前的预处理过滤配置。
          PREPARE: {
             # 根据目标包含的最小点数进行过滤。例如，'Car:5' 表示只选择至少包含5个点的Car目标。
             filter_by_min_points: ['Car:5', 'Pedestrian:5', 'Cyclist:5'],
             # 根据目标的难度等级进行过滤。-1 表示不过滤任何难度等级。
             filter_by_difficulty: [-1],
          }

          # 定义每个类别的采样数量。例如，'Car:20' 表示每个场景尝试额外采样20个Car目标。
          SAMPLE_GROUPS: ['Car:20','Pedestrian:15', 'Cyclist:15']

          # 点云的特征维度。这里是4 (x, y, z, intensity)。
          NUM_POINT_FEATURES: 4

          # 数据库中的点云是否是由虚拟激光雷达（FakeLiDAR）生成的。
          DATABASE_WITH_FAKELIDAR: False

          # 在放置采样的目标时，是否移除目标周围额外的宽度（用于避免与场景中原有目标重叠）。
          # [0.0, 0.0, 0.0] 表示不移除。
          REMOVE_EXTRA_WIDTH: [0.0, 0.0, 0.0]

          # 是否限制整个场景在采样后的总点数或目标数，防止内存溢出。
          LIMIT_WHOLE_SCENE: True

        # 4.2. 随机翻转
        # 对整个点云场景进行随机翻转。
        - NAME: random_world_flip
          # 指定可以翻转的轴。'x' 表示沿着x轴（左右方向）翻转。
          ALONG_AXIS_LIST: ['x']

        # 4.3. 随机旋转
        # 对整个点云场景进行随机旋转。
        - NAME: random_world_rotation
          # 旋转角度的范围（弧度）。这里是在 [-45°, 45°] 之间随机旋转。
          WORLD_ROT_ANGLE: [-0.78539816, 0.78539816] # -π/4 到 π/4

        # 4.4. 随机缩放
        # 对整个点云场景进行随机缩放。
        - NAME: random_world_scaling
          # 缩放比例的范围。场景会被一个在此范围内随机选择的因子进行整体缩放。
          WORLD_SCALE_RANGE: [0.95, 1.05]

# -----------------------------------------------------------------------------
# 5. 数据处理器配置 (DATA_PROCESSOR)
# 这些是在数据增强之后、将数据送入模型之前执行的一系列确定性处理步骤。
# 它们将原始点云数据转换为模型需要的格式（如体素Voxel）。
# -----------------------------------------------------------------------------

DATA_PROCESSOR:
    # 5.1. 过滤超出范围的点和包围框
    # 再次应用 POINT_CLOUD_RANGE 过滤点，并移除完全在该范围内的标注框。
    - NAME: mask_points_and_boxes_outside_range
      REMOVE_OUTSIDE_BOXES: True

    # 5.2. 打乱点的顺序
    # 对点云中的点进行随机打乱。
    - NAME: shuffle_points
      # 控制在不同阶段（训练/测试）是否启用。训练时启用可以防止模型学习到点的顺序信息。
      SHUFFLE_ENABLED: {
        'train': True,
        'test': False
      }

    # 5.3. 将点云转换为体素（Voxelization）
    # 这是将稀疏点云转换为密集张量的关键步骤，几乎所有现代3D检测模型都会使用。
    - NAME: transform_points_to_voxels
      # 体素的大小（x, y, z）。例如，[0.05, 0.05, 0.1] 表示每个体素在x和y方向上是5cm，z方向上是10cm。
      VOXEL_SIZE: [0.05, 0.05, 0.1]

      # 每个体素中最多包含的点数。超过此数量的点会被随机丢弃或采样。
      MAX_POINTS_PER_VOXEL: 5

      # 整个场景中最多允许的体素数量。这决定了模型输入张量的大小。
      # 训练时通常设置得小一些以节省显存，测试时可以设置得大一些以获得更精细的结果。
      MAX_NUMBER_OF_VOXELS: {
        'train': 16000,
        'test': 40000
      }
```

## pointrcnn.yaml

```yaml
# -----------------------------------------------------------------------------
# 1. 检测类别配置
# -----------------------------------------------------------------------------
# 模型需要检测的目标类别列表。
# 目前配置为只检测 'Car' 类别。
# 被注释掉的部分显示了原始配置支持检测 'Car', 'Pedestrian', 'Cyclist' 三个类别。
CLASS_NAMES: ['Car']
# CLASS_NAMES: ['Car', 'Pedestrian', 'Cyclist']

# -----------------------------------------------------------------------------
# 2. 数据配置 (DATA_CONFIG)
# 继承自基础数据集配置文件，并可以在此处进行覆盖和修改。
# -----------------------------------------------------------------------------
DATA_CONFIG:
    # 指定基础数据集配置文件的路径。
    # 该文件定义了数据集的根路径、点云范围、数据增强等基础信息。
    _BASE_CONFIG_: /home/zonlin/CRLFnet/src/site_model/src/LidCamFusion/OpenPCDet/tools/cfgs/dataset_configs/custom_dataset.yaml

    # （可能用于实时推理的）基础数据集配置文件路径。
    # 在许多情况下，它与训练时使用的配置相同。
    _BASE_CONFIG_RT_: /home/zonlin/CRLFnet/src/site_model/src/LidCamFusion/OpenPCDet/tools/cfgs/dataset_configs/custom_dataset.yaml

    # 数据处理器列表，定义了在数据送入模型前的处理步骤。
    # 这些步骤会覆盖或追加 _BASE_CONFIG_ 中定义的步骤。
    DATA_PROCESSOR:
        # 2.1. 过滤超出指定范围的点和包围框
        -   NAME: mask_points_and_boxes_outside_range
            # 是否移除完全在点云范围外的标注框。
            REMOVE_OUTSIDE_BOXES: True

        # 2.2. 对点云进行采样，以固定点数
        -   NAME: sample_points
            # 采样后的目标点数。训练和测试时都采样到 16384 个点。
            NUM_POINTS: {
                'train': 16384,
                'test': 16384
            }

        # 2.3. 打乱点的顺序
        -   NAME: shuffle_points
            # 控制在不同阶段（训练/测试）是否启用。训练时启用可以防止模型学习到点的顺序信息。
            SHUFFLE_ENABLED: {
                'train': True,
                'test': False
            }

# -----------------------------------------------------------------------------
# 3. 模型结构配置 (MODEL)
# 定义了 PointRCNN 模型的详细结构。
# -----------------------------------------------------------------------------
MODEL:
    # 指定使用的模型架构名称。
    NAME: PointRCNN

    # 3.1. 3D 骨干网络 (Backbone)
    # PointRCNN 使用 PointNet++ 作为其 3D 特征提取骨干网络。
    BACKBONE_3D:
        # 骨干网络的具体类型：多尺度分组的 PointNet++ (PointNet2MSG)。
        NAME: PointNet2MSG

        # 采样与聚合 (Set Abstraction, SA) 层的配置。
        # PointNet++ 通过多个 SA 层逐步下采样并提取更具表达力的特征。
        SA_CONFIG:
            # 每个 SA 层采样后的目标点数。
            NPOINTS: [4096, 1024, 256, 64]
            # 每个 SA 层中，对于每个点，在不同尺度下进行特征聚合的搜索半径。
            RADIUS: [[0.1, 0.5], [0.5, 1.0], [1.0, 2.0], [2.0, 4.0]]
            # 在每个搜索半径内，采样的邻居点数。
            NSAMPLE: [[16, 32], [16, 32], [16, 32], [16, 32]]
            # 每个 SA 层中，用于特征提取的多层感知机 (MLP) 的维度。
            # 列表的每一项对应一个尺度的半径。
            MLPS: [[[16, 16, 32], [32, 32, 64]],
                   [[64, 64, 128], [64, 96, 128]],
                   [[128, 196, 256], [128, 196, 256]],
                   [[256, 256, 512], [256, 384, 512]]]

        # 特征传播 (Feature Propagation, FP) 层的 MLP 配置。
        # FP 层将高层特征传播回低层，以进行更精细的预测。
        FP_MLPS: [[128, 128], [256, 256], [512, 512], [512, 512]]

    # 3.2. 点级头部 (Point Head)
    # PointRCNN 的第一阶段，在原始点云上预测目标的大致位置和类别。
    POINT_HEAD:
        # 点级头部的类型。
        NAME: PointHeadBox

        # 分类分支的全连接层维度。
        CLS_FC: [256, 256]
        # 回归分支的全连接层维度。
        REG_FC: [256, 256]

        # 是否是类别无关的预测。False 表示为每个类别单独预测。
        CLASS_AGNOSTIC: False

        # 在特征融合前是否使用点特征。对于 PointRCNN，此值通常为 False。
        USE_POINT_FEATURES_BEFORE_FUSION: False

        # 目标生成的配置。
        TARGET_CONFIG:
            # 在生成训练目标时，为真实框 (GT Box) 增加的额外宽度，用于捕捉更多上下文。
            GT_EXTRA_WIDTH: [0.2, 0.2, 0.2]
            # 包围框编码方式。
            BOX_CODER: PointResidualCoder
            # 包围框编码器的配置。
            BOX_CODER_CONFIG: {
                # 是否使用预定义的平均目标尺寸来辅助编码。
                'use_mean_size': True,
                # 每个类别的平均尺寸 [长, 宽, 高]。顺序与 CLASS_NAMES 对应。
                'mean_size': [
                    [3.9, 1.6, 1.56],  # Car
                    [0.8, 0.6, 1.73],  # Pedestrian (当前配置未使用)
                    [1.76, 0.6, 1.73]  # Cyclist (当前配置未使用)
                ]
            }

        # 损失函数配置。
        LOSS_CONFIG:
            # 回归损失函数类型。
            LOSS_REG: WeightedSmoothL1Loss
            # 各项损失的权重。
            LOSS_WEIGHTS: {
                'point_cls_weight': 1.0,    # 分类损失权重
                'point_box_weight': 1.0,    # 回归损失权重
                # 包围框编码中各个参数的损失权重 (dx, dy, dz, dx_size, dy_size, dz_size, rot_x, rot_y)
                'code_weights': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            }

    # 3.3. ROI 头部 (Region of Interest Head)
    # PointRCNN 的第二阶段，对第一阶段生成的候选框 (RoI) 进行进一步的精细分类和位置回归。
    ROI_HEAD:
        # ROI 头部的类型。
        NAME: PointRCNNHead

        # 是否是类别无关的 RoI 预测。
        CLASS_AGNOSTIC: True

        # RoI 点采样配置。
        ROI_POINT_POOL:
            # 在 RoI 周围扩展的额外宽度，用于采样更多相关点。
            POOL_EXTRA_WIDTH: [0.0, 0.0, 0.0]
            # 从每个 RoI 中采样的点数。
            NUM_SAMPLED_POINTS: 512
            # 深度归一化因子，用于将深度坐标归一化到 [-1, 1] 范围。
            DEPTH_NORMALIZER: 70.0

        # 用于提升 XYZ 坐标特征的全连接层。
        XYZ_UP_LAYER: [128, 128]
        # 分类分支的全连接层维度。
        CLS_FC: [256, 256]
        # 回归分支的全连接层维度。
        REG_FC: [256, 256]
        # Dropout 比例。
        DP_RATIO: 0.0
        # 是否使用批量归一化 (Batch Normalization)。
        USE_BN: False

        # 在 RoI 内进行特征聚合的 SA 层配置。
        SA_CONFIG:
            NPOINTS: [128, 32, -1] # -1 表示使用所有点
            RADIUS: [0.2, 0.4, 100] # 100 是一个很大的半径，确保能包含所有点
            NSAMPLE: [16, 16, 16]
            MLPS: [[128, 128, 128],
                   [128, 128, 256],
                   [256, 256, 512]]

        # 非极大值抑制 (NMS) 配置，用于在训练时筛选 RoI。
        NMS_CONFIG:
            TRAIN:
                NMS_TYPE: nms_gpu
                MULTI_CLASSES_NMS: False
                NMS_PRE_MAXSIZE: 9000
                NMS_POST_MAXSIZE: 512
                NMS_THRESH: 0.8
            TEST:
                NMS_TYPE: nms_gpu
                MULTI_CLASSES_NMS: False
                NMS_PRE_MAXSIZE: 9000
                NMS_POST_MAXSIZE: 100
                NMS_THRESH: 0.85

        # RoI 目标生成的配置。
        TARGET_CONFIG:
            # 包围框编码方式。
            BOX_CODER: ResidualCoder
            # 每张图像中采样的 RoI 数量。
            ROI_PER_IMAGE: 128
            # 前景 (Foreground) RoI 在采样中的比例。
            FG_RATIO: 0.5

            # 是否按类别采样 RoI。
            SAMPLE_ROI_BY_EACH_CLASS: True
            # 用于判断前景/背景的分数类型。
            CLS_SCORE_TYPE: cls

            # 分类任务中，判断为前景的 IoU 阈值。
            CLS_FG_THRESH: 0.6
            # 分类任务中，判断为背景的 IoU 阈值。
            CLS_BG_THRESH: 0.45
            # 分类任务中，用于难负样本挖掘的背景 IoU 下限。
            CLS_BG_THRESH_LO: 0.1
            # 难负样本在背景样本中的比例。
            HARD_BG_RATIO: 0.8

            # 回归任务中，判断为前景的 IoU 阈值。
            REG_FG_THRESH: 0.55

        # 损失函数配置。
        LOSS_CONFIG:
            # 分类损失函数。
            CLS_LOSS: BinaryCrossEntropy
            # 回归损失函数。
            REG_LOSS: smooth-l1
            # 是否使用角点损失正则化，以提升包围框的定位精度。
            CORNER_LOSS_REGULARIZATION: True
            # 各项损失的权重。
            LOSS_WEIGHTS: {
                'rcnn_cls_weight': 1.0,    # 分类损失权重
                'rcnn_reg_weight': 1.0,    # 回归损失权重
                'rcnn_corner_weight': 1.0, # 角点损失权重
                # 包围框编码中各个参数的损失权重 (dx, dy, dz, dx_size, dy_size, dz_size, rot)
                'code_weights': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            }

    # 3.4. 后处理配置 (POST_PROCESSING)
    # 定义了模型推理后，如何从网络输出中筛选和格式化最终检测结果。
    POST_PROCESSING:
        # 用于评估召回率的阈值列表。
        RECALL_THRESH_LIST: [0.3, 0.5, 0.7]
        # 检测结果的分数阈值，低于此阈值的结果将被过滤。
        SCORE_THRESH: 0.1
        # 是否输出原始的预测分数。
        OUTPUT_RAW_SCORE: False

        # 使用的评估指标。
        EVAL_METRIC: kitti

        # 最终检测结果的 NMS 配置。
        NMS_CONFIG:
            # 是否对每个类别分别进行 NMS。
            MULTI_CLASSES_NMS: False
            # NMS 类型。
            NMS_TYPE: nms_gpu
            # NMS 阈值。
            NMS_THRESH: 0.1
            # NMS 前的最大候选框数量。
            NMS_PRE_MAXSIZE: 4096
            # NMS 后的最大检测框数量。
            NMS_POST_MAXSIZE: 500

# -----------------------------------------------------------------------------
# 4. 优化策略配置 (OPTIMIZATION)
# 定义了模型训练过程中的优化器、学习率策略等超参数。
# -----------------------------------------------------------------------------
OPTIMIZATION:
    # 每个 GPU 的批次大小。
    BATCH_SIZE_PER_GPU: 2
    # 总的训练轮数 (Epochs)。
    NUM_EPOCHS: 80

    # 使用的优化器类型。'adam_onecycle' 是 Adam 优化器结合 OneCycle 学习率调度策略。
    OPTIMIZER: adam_onecycle
    # 初始学习率。
    LR: 0.01
    # 权重衰减 (L2 正则化) 系数，用于防止过拟合。
    WEIGHT_DECAY: 0.01
    # 动量，用于加速收敛。
    MOMENTUM: 0.9

    # OneCycle 策略中的动量值。
    MOMS: [0.95, 0.85]
    # 学习率上升阶段所占的训练周期比例。
    PCT_START: 0.4
    # 初始学习率的除法因子。
    DIV_FACTOR: 10
    # 学习率衰减的轮数节点。
    DECAY_STEP_LIST: [35, 45]
    # 学习率衰减因子。
    LR_DECAY: 0.1
    # 学习率的下限。
    LR_CLIP: 0.0000001

    # 是否启用学习率热身 (Warmup)。
    LR_WARMUP: False
    # 热身的轮数。
    WARMUP_EPOCH: 1

    # 梯度裁剪的阈值，用于防止梯度爆炸。
    GRAD_NORM_CLIP: 10
```

## 目录结构

```textile
custom
├── ImageSets
│   ├── test.txt
│   ├── train.txt
├── testing
│   ├── velodyne
├── training
│   ├── label_2
│   ├── velodyne
```

这个目录结构抓得很准，正是 OpenPCDet 适配激光雷达 3D 目标检测的标准数据组织形式，核心是按 “数据集划分 + 数据类型” 分类存储文件。

### 核心目录功能总览

该结构专门为基于 Velodyne 激光雷达的 3D 检测任务设计，清晰区分训练 / 测试数据、样本划分规则和标注信息，符合 OpenPCDet 对数据集的读取规范。

---

### 各目录详细说明

#### 1. 根目录 custom

自定义数据集的根文件夹，所有相关数据和配置都集中在此，名称可自定义（如 kitti、waymo 等），OpenPCDet 通过配置文件指定该根目录路径。

#### 2. ImageSets 目录

存储数据集的**划分规则文件**，定义哪些样本用于训练、哪些用于测试。

- train.txt：包含训练集样本的索引（无后缀），每行一个样本 ID，对应 training 目录下的同名数据文件。
- test.txt：包含测试集样本的索引，对应 testing 目录下的同名数据文件，无标注信息（仅用于推理）。

#### 3. testing 目录

存储测试集的原始数据（无标注，仅用于模型推理）。

- velodyne：存放测试集的激光雷达点云文件，格式通常为.bin（二进制文件），每个文件对应一个样本的 3D 点云数据（包含 x、y、z、反射强度等信息）。

#### 4. training 目录

存储训练集的 “数据 + 标注”，供模型训练时读取。

- label_2：存放训练集的 3D 目标检测标注文件，格式为.txt，每个文件与 velodyne 目录下的点云文件一一对应。
  
  标注内容包括目标类别（如 car、pedestrian）、3D 边界框坐标、旋转角度、置信度等。

- velodyne：与 testing/velodyne 格式一致，存放训练集的激光雷达点云.bin 文件，是模型训练的核心输入数据。

---

### 关键补充

- 缺失目录说明：标准结构中通常还会有 “calib”（相机 - 激光雷达标定文件）、“image_2”（相机图像）目录，若未列出可能是该数据集仅使用激光雷达数据，或已省略非必要模态。
- 文件关联逻辑：样本 ID 是核心关联键，例如 train.txt 中的 “000001”，对应 training/velodyne/000001.bin 和 training/label_2/000001.txt。

## 这条命令是 **OpenPCDet** 框架中用于

生成自定义数据集信息文件(.pkl)的关键指令，目的是将原始点云、标注数据整理成模型训练 / 测试时能快速加载的结构化格式。以下是每个参数的详细解释：

```bash
python -m pcdet.datasets.custom.custom_dataset create_custom_infos tools/cfgs/dataset_configs/custom_dataset.yaml
```

核心功能：调用 OpenPCDet 的自定义数据集处理模块，读取 `custom_dataset.yaml` 配置，遍历原始数据集（点云文件、标注文件等），提取关键信息（如点云路径、标注框坐标、类别、校准参数等），最终生成 `.pkl` 格式的数据集信息文件（如 `custom_infos_train.pkl`、`custom_infos_val.pkl`），供后续模型训练 / 推理使用。

### 各参数详细解释

#### 1. `python -m`

- **作用**：以「模块方式」运行 Python 脚本，而非直接运行 `.py` 文件。
- **优势**：
  - 自动将当前工作目录添加到 Python 路径（`sys.path`），避免导入自定义模块时出现「模块找不到」错误（尤其适用于 OpenPCDet 这种多目录结构的项目）。
  - 确保模块的导入逻辑与项目整体一致（比如 `pcdet.datasets.custom.custom_dataset` 能正确导入 `pcdet` 核心模块）。

#### 2. `pcdet.datasets.custom.custom_dataset`

- **作用**：指定要运行的核心模块，是 OpenPCDet 中处理「自定义数据集」的入口模块。
- **模块功能**：
  - 定义了自定义数据集的读取逻辑（如何解析点云文件、标注文件、校准参数等）。
  - 包含 `create_custom_infos` 函数（命令的核心执行逻辑），负责遍历数据集、提取信息、生成 `.pkl` 文件。
- **依赖**：模块的功能依赖 `custom_dataset.yaml` 配置文件中的参数（如数据集路径、类别、点云范围等）。

#### 3. `create_custom_infos`

- **作用**：指定模块中要执行的「具体函数 / 命令」，是生成数据集信息文件的核心逻辑入口。
- **函数功能**：
  - 读取 `custom_dataset.yaml` 中的数据集配置（如 `DATA_PATH`、`CLASS_NAMES`、`DATA_SPLIT` 等）。
  - 遍历训练集 / 测试集的原始数据（点云文件：如 `.bin`/`.pcd`，标注文件：如 `.txt`）。
  - 对每个样本提取关键信息：
    - 点云文件的绝对路径。
    - 标注框（`gt_boxes`）：3D 坐标（x/y/z）、尺寸（长 / 宽 / 高）、旋转角、类别。
    - （可选）相机 - 激光雷达校准参数（`calib`）、图像路径（若涉及多模态数据）。
  - 将所有样本的信息整理成列表，保存为 `.pkl` 格式（二进制文件，加载速度远快于原始文本文件）。
- **输出**：生成的 `.pkl` 文件会保存在 `DATA_PATH` 指定的数据集根目录下（或 `INFO_PATH` 配置的路径），例如：
  - `custom_infos_train.pkl`（训练集信息）
  - `custom_infos_val.pkl`（验证集信息）

#### 4. `tools/cfgs/dataset_configs/custom_dataset.yaml`

- **作用**：指定「数据集配置文件」的路径，为 `create_custom_infos` 函数提供必要的参数支撑。
- **配置文件核心参数（与命令相关）**：
  - `DATA_PATH`：数据集根路径（原始点云、标注文件所在目录）。
  - `CLASS_NAMES`：数据集包含的目标类别（如 `['Car', 'Pedestrian']`）。
  - `DATA_SPLIT`：训练集 / 测试集的划分方式（如 `train: train.txt`、`test: val.txt`，指定样本列表文件）。
  - `INFO_PATH`：生成的 `.pkl` 信息文件的保存路径（默认在 `DATA_PATH` 下）。
  - `POINT_CLOUD_RANGE`：点云有效范围（用于过滤无效点，影响信息提取时的点云处理）。
  - 其他辅助参数：如点云文件格式（`.bin`/`.pcd`）、标注文件格式（如 KITTI 格式 `.txt`）等。
- **必要性**：配置文件是命令执行的「依据」，若缺少或配置错误（如路径不对、类别不匹配），会导致信息提取失败。

### 总结：命令执行流程

1. `python -m` 以模块方式启动 `pcdet.datasets.custom.custom_dataset` 模块。
2. 模块接收 `create_custom_infos` 命令，触发核心函数执行。
3. 函数读取 `custom_dataset.yaml` 配置，获取数据集路径、类别、划分等参数。
4. 遍历原始数据集，提取每个样本的点云路径、标注框、类别等信息。
5. 将所有样本信息整理成结构化数据，保存为 `.pkl` 格式的信息文件。
6. 输出文件供后续模型训练（如 `train.py` 会读取这些 `.pkl` 文件快速加载数据）。

### 注意事项

1. 配置文件 `custom_dataset.yaml` 中的 `DATA_PATH` 必须正确指向原始数据集根目录（否则无法找到点云 / 标注文件）。
2. 原始数据集的目录结构、文件格式需与配置文件匹配（如点云文件在 `DATA_PATH/points/` 下，标注文件在 `DATA_PATH/labels/` 下）。
3. 生成的 `.pkl` 文件无需手动修改，后续训练 / 推理会自动读取。
4. 
