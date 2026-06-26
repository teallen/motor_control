# 08 实时安全练习

对应教材：[course/08_realtime_safety.md](../course/08_realtime_safety.md)

## 实验 1：周期抖动与故障

运行：

```bash
./build/examples/sim_realtime_jitter_safety
```

画出：

- `dt_s`
- `jitter_s`
- `measured_current_a`
- `is_enabled`
- `is_fault`
- `watchdog_miss`

回答：

1. 系统在什么时候进入 fault？
2. fault 后是否保持 enabled？
3. fault cleared 后为什么不应直接回到 enabled？

## 实验 2：看门狗阈值

修改 [examples/sim_realtime_jitter_safety.cpp](../examples/sim_realtime_jitter_safety.cpp) 中 `watchdogMiss` 的阈值。

回答：

1. 阈值太紧会有什么问题？
2. 阈值太松会有什么风险？
3. 实际系统中看门狗应该监控哪些信号？

## 实验 3：过流保护

把 `commandCurrent` 的阶跃从 9 A 改成 7 A。

回答：

1. 是否还会因过流进入 fault？
2. 如果没有 fault，系统是否仍可能因热限制降额？
3. 过流和过温保护的时间尺度有什么区别？
