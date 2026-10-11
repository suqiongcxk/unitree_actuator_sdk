# V46/model_1250 RK3588 部署验证记录

日期：2026-10-01

## 部署对象

```text
目录: model_deliveries/creeper_v46_model1250
ONNX: creeper_model_v46_model1250_actor.onnx
SHA256: 2893426e5d3cf0c42dbbaef6236cbc3b38e889f89fda8acc8ce464197f913cfc
```

工程默认模型已由 V45/model_999 切换为 V46/model_1250。仍可通过命令行
`--onnx <path>` 临时覆盖默认模型。

## 接口核对

- 输入：`obs`，`float32[1,48]`；
- 输出：`actions`，`float32[1,12]`；
- 策略频率：50 Hz；
- action scale：0.25；
- `q_des = default_joint_position + 0.25 * raw_action`；
- observation、action关节顺序与当前实机代码一致；
- ONNX文件实际SHA256与侧车及manifest一致。

## 验证边界

训练端交付报告记录 Isaac 和 MuJoCo 为 PASS，Hardware 为 NOT_TESTED。报告声明V46在
训练与离线扫描中将确定性`q_des`约束在实机机械安全范围内，但这不能替代RK3588实机
验证。首次实机运行仍须保留现有关节目标校验、状态估计、通信和电机错误保护，并按
零命令、低速单轴、组合命令的顺序逐步验收。

交付manifest还引用了未随六件核心文件上传的`verification/`、`configs/`、`assets/`
和`evidence/`材料。这不影响当前ONNX加载和golden-vector回放，但这些外部证据未在
本机逐文件复核。
