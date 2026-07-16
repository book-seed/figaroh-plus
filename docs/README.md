# FIGAROH Documentation

## 指南文档

| 文档 | 内容 |
|------|------|
| [developer-guide.md](developer-guide.md) | 开发者总入口：安装、架构、配置、API |
| [fourier-optimal-trajectory.md](fourier-optimal-trajectory.md) | 傅里叶激励轨迹优化：环境搭建、配置、运行、排错 |
| [architecture.md](architecture.md) | 全项目类关系 + 架构文档 |

## 构建和查看 Sphinx 文档

```bash
pixi run python -m pip install sphinx sphinx-rtd-theme
cd docs && make html
```

本地查看：

```bash
cd build/html && python -m http.server 8000
# 浏览器打开 http://localhost:8000
```
