# 一次性材料事件 KKT / 功根参考原型

这是一个 **work-only、纯 JavaScript** 的力学参考实现，不修改正式模拟器。它解决一个很窄但关键的问题：在一次切削事件中，材料阻力冲量和叉子/靶子/地面等结构接触必须属于同一个约束解，不能先算材料、再投影结构，或反过来投影两遍。

## 模型和符号

- `qFree`：该事件时刻包含外力后的自由广义速度。
- `Minv`：对称正定广义逆质量矩阵。
- 结构行 `a_i`：采用 `a_i · q + bias_i >= 0`，结构冲量为 `+a_i^T gamma_i`，`gamma_i >= 0`。
- 材料行 `j`：`j · q > 0` 表示工作面仍在切入；材料阻力只施加一次，为 `-j^T lambda`。
- 本参考版结构接触明确为 **无摩擦单边约束**。叉靶、靶地、叉地的法向行都可同时传入；摩擦必须由生产版以摩擦模式枚举扩展，不能在本原型外再补一次切向投影。

固定任意 `lambda` 后，原型枚举结构主动集并一次性求解：

```text
qPost = qFree + Minv · (A^T gamma - j^T lambda)
A qPost + bias >= 0
gamma >= 0
gamma_i (a_i qPost + bias_i) = 0
```

各主动集在 `lambda` 上都是仿射区间。实现会解析求出这些区间，在每个区间精确积分

```text
D(lambda) = integral_0^lambda max(0, j · qPost(p)) dp
```

所以结构主动集切换不会被固定采样或一次 Simpson 积分跨过去。随后解最小正根：

```text
D(lambda) - Uc · width · freshArea(lambda, sameTrial) = 0
```

`freshArea` 必须是纯函数，并返回固定事件片内由同一个 `lambda/qPost/activeIds` 得到的原生新切面积。对于阻力增大后的同一事件片，它必须连续且不增；不满足时只能缩短/细分事件片。

## 生产接口

```js
const prepared = prepareMaterialEvent({
  qFree,
  Minv,
  structuralRows: [
    { id: 'fork-target:0', row: [...], bias: 0 },
    { id: 'target-floor:0', row: [...], bias: 0 },
    { id: 'fork-floor:0', row: [...], bias: 0 },
  ],
  materialRow: j,
  specificCuttingEnergy: Uc,
  width,
  freshArea: (lambda, trial) => ({
    area: computeVirginAreaFromTheSameTrial(trial),
    payload: preparedRemainingGeometry,
  }),
});
```

成功结果绑定：

- 唯一接受的 `lambda`；
- 同一次 KKT 的 `qPost`、结构冲量和 `activeIds`；
- 同一 trial 生成的 `freshArea` 与几何 payload；
- 分段材料功和绑定签名。

生产提交应把 `qPost` **直接写入**所有相关刚体，并提交绑定的几何 payload。不得再调用一次接触求解器，不得拿零冲量端点和根端点做平均速度，也不得在提交阶段重新生成面积。`commitPreparedEvent` 给出了最小事务适配器：任何中途失败都会恢复提交前快照，并核对可序列化状态完全一致。

失败口径：

- `unaffordable-slice`：停止冲量的最大可用功仍小于新切体积功。只返回“建议最大面积比例”供调用者缩短事件片；**绝不缩放冲量或材料参数**。
- `solver-domain-stop`：主动集、面积单调性、确定性或数值根不成立。调用者只能细分事件或安全停止。
- `non-compressive-event`：联立结构约束后工作面没有正切入速度。

## 已覆盖测试

运行：

```powershell
node work/material-kkt-event/run-tests.js
```

测试覆盖结构夹持、材料路径中的主动集切换、付不起的事件片、非单调几何域停止、同一冲量/速度/主动集/几何绑定、零偏置能量闭合、部分提交故障回滚和重复运行确定性。报告写入 `test-report.json`。

## 有意保留的边界

- 结构法向行必须先去重并保持线性独立；奇异主动集会被跳过，多个不同速度解会域停止。
- 当前没有 Coulomb 摩擦。生产扩展必须把 stick / `+slide` / `-slide` 作为同一个 KKT 的离散模式枚举，而不是事后摩擦修正。
- `freshArea` 的几何构造、TOI 和剩余时间重演由上层负责；本原型只规定它们必须使用接受根的同一 trial。
- 非零 `bias` 可求解并积分，但只有全部 `bias=0` 时才启用 `K(0)-K(lambda)=D(lambda)` 的严格能量恒等审计；非零 Moreau 偏置还需要单列结构约束功。

