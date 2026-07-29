# Tasks: sink-backend-into-fourier-strategy

- [x] 移除 BaseOptimalTrajectory 中 _backend 赋值与 casadi import
- [x] FourierOptimizationStrategy.solve 内部按需构造 CasadiBackend
- [x] CasadiBackend 补 TODO 标注未来模块级共享单例
- [x] 运行相关测试确认行为不变

注: test_fourier_e2e 的 test_solve_produces_results 在 baseline (HEAD) 上即已失败
(cs.cholesky API 不兼容, CasADi 版本问题), 与本次下沉无关, 非本次引入回归。
