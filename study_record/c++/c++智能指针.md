# c++智能指针

## 为什么你的代码不行？

- 你从 vector 获取了智能指针 `tmp`
- 删除了 vector 中的元素，但 `tmp` 仍然持有该指针
- 然后你尝试手动 `delete tmp;`，这会：
  - 对于 `std::shared_ptr`：编译错误，因为它没有定义 `operator delete`
  - 对于 `boost::shared_ptr`：可能运行时崩溃，因为你绕过了引用计数机制
