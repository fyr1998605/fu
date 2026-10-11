# 多人 Git 工作流

稳定分支通过 PR 合并，不直接 push，不强制推送。新工作从最新目标分支创建
`feature/xxx`、`bugfix/xxx` 或 `gitfix/xxx`，不能从其他尚未合并的功能分支开始。

本仓库同时维护多个版本：远程默认分支为 `ros2`；本次 ROS1 能量机关改动基于
`ros1-auto_aim-fu`。该版本的 PR 应以 `ros1-auto_aim-fu` 为目标。
`main` 是另一个旧版本，跨版本迁移必须单独评估，不能直接覆盖。

以下以本次目标分支为例：

```bash
git switch ros1-auto_aim-fu
git pull --rebase origin ros1-auto_aim-fu
git switch -c feature/your-feature
# 开发并检查具体改动，不提交构建产物、日志或凭据。
git status
git add <本次修改的文件>
git commit -m "feat: 描述改动"
git push -u origin feature/your-feature
```

创建 PR，说明问题、改动、验证结果与限制，并指定队友 Review。
运行 `bash scripts/test_buff.sh` 可执行能量机关预测和时序回归；需要 ROS Noetic、
OpenCV、Eigen、CMake 和 Python。GitHub Actions 执行同一脚本。
该 CI 不代替 Fu 整体编译、相机串口测试或实机击打验证。

至少一位队友 Approve、CI 通过且没有待处理的修改请求后，确认功能分支仍基于
最新目标分支，再 Squash merge。未审核时保持 PR 打开，不用管理员权限绕过流程。

```bash
git switch ros1-auto_aim-fu
git pull --rebase origin ros1-auto_aim-fu
git branch -d feature/your-feature
git push origin --delete feature/your-feature
```

Squash merge 后 `git branch -d` 可能因历史不包含原提交而拒绝。先核对 PR 已合并，
确认分支没有额外提交，再删除本次分支。

更新自己的功能分支时可以 rebase；推送过的分支只用 `--force-with-lease`，且要与
同分支协作者协调。公共稳定分支禁止强推。解决冲突时保留双方有效改动，不确定的
业务语义交由相关作者确认。

2026-10-11 查询 GitHub 分支保护接口返回 403，提示本私有仓库需要 GitHub Pro
或改为公开仓库才能启用此功能。因此当前审核与 CI 条件由团队流程执行，尚未
由 GitHub 分支保护强制约束。本次不修改仓库可见性或账户套餐。
