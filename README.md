# M20 / D1 二维跟随与手机控制

保留 `bernardleex526/deamon2` 的点云转扫描、D1 跟随算法与 M20 Cmd 25 速度桥，
增加统一网页/Android 控制入口与 GOS 一键脚本。原 `deamon` 三维实现完整保留在
`legacy/m20_follow_control`，由 `COLCON_IGNORE` 排除。

## 使用

完成一次性编译和 NOS SSH 配置后，在 GOS：

```bash
cd /home/user/m20_ws/src/jie_deamon
bash scripts/m20ctl preview
```

手机访问 `http://<GOS可达IP>:8080`；另一个终端执行 `bash scripts/m20ctl token`
取得口令。网页支持雷达选点、按住方向控制、开始跟随、停止以及状态/拦截原因显示。

- 默认演练：不打开 AOS 控制 socket，不 arm，可预览原始算法速度。
- 实机入口 `bash scripts/m20ctl live --commissioned` 仅用于完成验收与原厂控制权移交之后。
- 网页/Android 共用唯一控制权、后台状态/限速/超时/近障门控，断连后不自动恢复。
- Android 手机可直接用网页；原生 App 需升级为认证、序号与心跳协议，旧 D1 App 不直接兼容。
- D1 姿态/步态动作尚未完成 M20 验证映射，明确拒绝，继续使用原厂控制器。

**本次不宣称实机跟随验收通过。** 原记录中的异常横向输出仍需排查；本次未修改算法数学逻辑。

[完整部署、命令、Android 协议与验收说明](docs/OPERATOR.md)

## 数据链路

```text
/LIDAR/POINTS -> pointcloud_to_laserscan -> /m20/scan -> D1 tracker
                                                         |
                       session-tagged follow velocity ---+
                                      |
phone HTTP / Android UDP -> exclusive operator lease -> M20 guard -> AOS UDP
```

M20 默认使用新认证网页，旧 C++ Web/Android/D1 动作服务器不参与 M20 链路。
系统仍不替代原厂 SLAM 或四足运控。

## 验证

```bash
python3 -m unittest discover -s test -v
```

CI 定义包含 Foxy/Humble 编译、跟随回归测试和隔离 ROS/HTTP smoke。
请以实际 CI 结果为准；离线单元测试不等于机器人验收。

历史适配证据见 [deamon2 原说明](docs/DEAMON2_ORIGINAL_README.md)、
[实机 dry-run 记录](docs/M20_DRY_RUN_2026-09-08.md)。历史文档中的旧网页禁用规则与
手动 ROS 操作流程以本页及 `OPERATOR.md` 的新网关流程为准。

## 来源与许可证

基于 6-robot/jie_deamon 与 bernardleex526/deamon2（MIT）。保留原作者信息。
