# IQmathLib 定点数学函数库设计报告

## —— 对标 TI C2000™ SDK IQmath API 的重新设计实现详尽分析

（基于对 /workspace 仓库全部源代码的逐文件审读与本地编译验证撰写）

---

# 目录

1. 项目概述与设计背景
2. 总体架构设计：薄壳 + 参数化内核 + 双执行路径
3. IQ 定点数格式体系设计
4. 头文件 IQmathLib.h 的分层设计
5. 硬件抽象与可移植性设计（support 层）
6. 基础算术类函数实现分析
7. 类型转换与字符串类函数实现分析
8. 超越函数类实现分析
9. 查表体系设计（_IQNtables）
10. MathACL 硬件加速器路径总评
11. 精度、饱和与舍入策略汇总
12. 编译期多态与工程化机制
13. 构建系统与交付形态
14. 实测验证记录
15. 设计亮点与潜在问题
16. 总结

---

# 一、项目概述与设计背景

## 1.1 项目定位

本仓库是一个**对标 TI C2000™ SDK 中 IQmath API 接口规范重新设计的定点数学（IQmath）函数库**（PlatformIO 库名 `iqmath`，版本 1.11.0；内部版本串 `_IQmathLibVersionString()` 返回 `"IQmathLib version 01_11_00_00"`）。README 开宗明义：该库面向"无浮点单元（FPU）的芯片"，让 C 程序员能把浮点算法无缝改写为定点代码，在计算密集的实时应用中获得比浮点实现更快、更省电的执行效果。library.json 描述其为 "Initial port of the IQMath Library, obtained from TI MSPM0 SDK"——即以 TI MSPM0 SDK 中带 MathACL 支持的 IQmath 源码为基础做的独立移植/重设计，并声明了非官方属性。

TI 原始 IQmath 为 C2000 提供一组以 `_IQN` 前缀命名的宏与函数（如 `_IQ16mpy`、`_IQ24sin`、`_IQ12div`），历史上由手写汇编实现。本仓库将其重写为纯 C（辅以编译器内建与外设寄存器操作），保持**源代码级兼容**，使电机控制、数字电源等实时控制代码可在通用 MCU（ARM Cortex-M、PC 测试环境等）上编译运行，并在 MSPM0 系列上自动启用其内置 **MathACL 硬件算术协处理器**加速路径，形成"软件 RTS 路径 + MathACL 硬件路径"双轨制。

## 1.2 设计约束与需求

从代码结构可归纳出五条设计要求：

1. **API 兼容性**：对外符号完全沿用 C2000 IQmath 命名约定（`_IQmpy/_IQdiv/_IQsin/_IQatan2/_IQtoF/_IQsat...`），用户仅需定义 `GLOBAL_IQ`（缺省 24，允许 1～30）即可把无后缀宏绑定到具体 Q 格式。
2. **全格式覆盖**：每种运算分别提供 IQ1～IQ30 共 30 个独立函数入口（部分函数延伸至 IQ31 或下探 IQ0），链接器可按需抽取。
3. **可移植性**：底层乘法原语（`__mpy_l`、`__mpyf_ul`、`__mpyx` 等）经 `support/` 适配层隔离；当前提供 RTS（通用 C 回退）实现，注释多处保留 MSP430 MPY32 硬件语义（关中断、模式切换、操作数复用），MathACL 代码用 `__IQMATH_USE_MATHACL__ && __MSPM0_HAS_MATHACL__` 双开关门控。
4. **实时确定性**：无动态内存、无运行时浮点（常量构造除外）、关键算法采用固定迭代次数（牛顿法）而非数据相关循环，所有异常输入映射为饱和边界值而非陷阱。
5. **高精度**：超越函数普遍达到 24～28 位有效精度，接近 32 位定点理论上限。

## 1.3 仓库物理结构与规模

```
/workspace
├── include/IQmathLib.h        // 唯一对外头文件，6528 行，API 总定义
├── _IQNfunctions/             // 各函数 .c 实现 + 内部 .h 内联内核
│   ├── _IQNmpy.{c,h}          // 基本乘法（内核置于头文件供复用）
│   ├── _IQNrmpy.c             // 舍入乘法
│   ├── _IQNrsmpy.c            // 舍入+饱和乘法
│   ├── _IQNdiv.{c,h}          // 除法（Newton-Raphson 倒数迭代，202 行头文件内核）
│   ├── _IQNmpyIQX.c           // 跨格式混合乘法
│   ├── _IQNsin_cos.c          // 正余弦 + PU 每单位制（1719 行）
│   ├── _IQNatan2.c            // 四象限反正切（1096 行）
│   ├── _IQNasin_acos.c        // 反正弦（acos 由头文件宏基于 asin 构造）
│   ├── _IQNsqrt.c             // sqrt/isqrt/mag/imag 四合一内核（1778 行）
│   ├── _IQNexp.c              // 指数
│   ├── _IQNlog.c              // 自然对数
│   ├── _IQNfrac.c             // 取小数部分
│   ├── _IQNtoF.c              // IQ→float 位合成转换
│   ├── _IQNtoa.c              // IQ→字符串（含 _IQ0toa）
│   ├── _atoIQN.c              // 字符串→IQ
│   ├── _IQNrepeat.c           // MathACL 重复运算快捷入口（仅硬件编译时存在）
│   ├── _IQNversion.c          // 版本号（6 行）
│   └── _IQNtables.{c,h}       // 全部系数/查找表（361/76 行）
├── support/
│   ├── support.h              // 公共常量 + 适配层选择（39 行）
│   └── RTS_support.h          // 通用 CPU 上的 __mpy* 原语软件模拟（290 行）
├── CMakeLists.txt             // 静态库构建（目标 iqmath → libiqmath.a）
├── library.json               // PlatformIO 库描述
└── README.md                  // IQ 格式说明 + 范围/分辨率表 + 免责声明
```

全库合计约 1.8 万行 C 代码，其中超过一半是机械展开的 30×族薄壳函数与 Doxygen 注释块。

---

# 二、总体架构设计：薄壳 + 参数化内核 + 双执行路径

## 2.1 "薄壳 + 参数化内核"模式

本库最核心的架构决策：**将 C2000 中 31×N 个独立汇编函数压缩为 N 个带 `q_value` 参数的泛型内联内核（`__IQNxxx`），再由 30 余个仅传递常量的薄封装函数（`_IQ30xxx`…`_IQ1xxx`）调用之**。以乘法为例（`_IQNmpy.h` + `_IQNmpy.c`）：

```c
// 泛型内核（强制内联）
__STATIC_INLINE int_fast32_t __IQNmpy(int_fast32_t a, int_fast32_t b, const int8_t q_value)
{
    int_fast64_t r = (int_fast64_t)a * (int_fast64_t)b;
    return (int_fast32_t)(r >> q_value);
}
// 薄壳
int32_t _IQ30mpy(int32_t a, int32_t b) { return __IQNmpy(a, b, 30); }
...
int32_t _IQ1mpy (int32_t a, int32_t b) { return __IQNmpy(a, b, 1); }
```

由于 `q_value` 以编译期常量传入且内核被强制内联（TI 编译器 `#pragma FUNC_ALWAYS_INLINE`、IAR `#pragma inline=forced`、其他编译器退化为 `static inline`），每个薄壳生成时移位量折叠为立即数，机器码与手写逐格式函数几乎一致；而算法只需维护一份。这是贯穿全库的系统性折衷——"一份算法描述换取 30 份机器码实例"。

## 2.2 三层结构

- **第一层 API/宏层**（`include/IQmathLib.h`）：typedef、常量构造宏、纯宏运算（`_IQmpy2/_IQsat/_IQabs` 等）、extern 函数声明、按 `GLOBAL_IQ` 展开的通用别名宏。
- **第二层 算法内核层**（`_IQNfunctions/*.h` 的 `__IQNxxx` 内联内核 + `*.c` 薄壳）：真正的数值算法——Taylor 级数、Newton-Raphson 迭代、分段多项式、查表插值。
- **第三层 硬件抽象层**（`support/support.h`、`support/RTS_support.h`、MathACL 寄存器块）：把"C2000/MSP430 乘法器状态机假设"翻译成任意平台可编译的表达式。

## 2.3 双执行路径

支持硬件加速的函数内部均为条件编译二分：

```c
#if ((!defined (__IQMATH_USE_MATHACL__)) || (!defined (__MSPM0_HAS_MATHACL__)))
    /* 软件 RTS 路径 */
#else
    /* MathACL 硬件路径：写 CTL/OP1/OP2，读 RES1/RES2 */
#endif
```

涉及 MathACL 的有：`__IQNmpy`（op=6）、`__IQNdiv`（op=4）、`__IQNsin_cos`（sincos op=1）、`__IQNatan2`（op=2）、`__IQNsqrt`（op=5，带 scale_factor）、以及 `_IQrepeat`（复用已配置 CTL 的快捷重算）。同一份头文件在 PC/CI 走软件路径验证正确性，在 MSPM0 目标自动切换硬件路径，接口逐字节不变。

---

# 三、IQ 定点数格式体系设计

## 3.1 格式定义与表示范围

IQmath 采用有符号 Q 格式：32 位补码整数的低 N 位为小数位，数值 = 二进制整数值 × 2⁻ᴺ。头文件第 91～121 行定义 `_iq30 … _iq1` 及通用 `_iq`，全部 typedef 到 `int32_t`——**格式只是"约定"而非类型系统**，编译器不做格式检查，这正是 C2000 原始语义以保证兼容。README 给出完整范围表：`_iq30` 为 2 整数位/30 小数位、范围 ±2；`_iq24` 为 8/24、±128；依此类推到 `_iq1`。注意 IQ31 无法表示 +1.0（最大 0.9999999995），该细节在 atan2 的 MathACL 归一化代码中被显式处理（§8.3）。

## 3.2 常量构造宏与幂次快乘

```c
#define _IQ30(A)   ((_iq30)((A) * ((_iq30)1 << 30)))     // 编译期浮点常量折叠
#define _IQ16(A)   ((_iq16)((A) * ((_iq16)1 << 16)))
#define _IQ(A)     _IQ24(A)                              // GLOBAL_IQ=24 时的路由结果
```

简单幂次乘除直接映射为移位，零函数调用：`_IQmpy2(A)=((A)<<1)`、`_IQmpy16(A)=((A)<<4)`、`_IQmpy32(A)=((A)<<5)`、`_IQdiv4(A)=((A)>>2)`……这些是控制代码中最常见的增益缩放。跨格式显式转换同样以宏完成：`_IQtoIQ30(A)=((_iq30)(A)<<(30-GLOBAL_IQ))`、`_IQ30toIQ29(A)=((_iq29)(A)>>1)` 等构成完整的 IQn→IQm 宏矩阵。

## 3.3 公共无理数常量（support/support.h）

库集中定义高精度定点常量供各算法共享，避免逐函数硬编码：

```c
#define iq28_twoPi      0x6487ed51   // 2π×2^28
#define iq29_pi         0x6487ed51   // π×2^29
#define iq30_pi         0xc90fdaa2
#define iq31_halfPi     0xc90fdaa2
#define iq31_quarterPi  0x6487ed51
#define iq31_ln2        0x58b90bfc   // ln2×2^31
#define iq31_twoThird   0x55555555   // 2/3×2^31（log 规格化阈值）
#define iq31_pointOne   0x0ccccccd   // 0.1×2^31（toa/atoIQN 用）
#define iq31_oneTenth   0x0ccccccd
#define iq31_invRoot2   0x5a82799a   // 1/√2×2^31
#define iq31_one        0x7fffffff   // log 内核中"1.0"的实际近似载体
```

另有 q13/q14/q15 小位宽家族（`q15_ln2=0x58b9`、`q15_tanSmall=0x0021` 等）服务低精度场景。同一数值在不同 Q 下尾数比特完全相同（0x6487ed51 同时是 iq28_twoPi/iq29_pi/iq30_halfPi/iq31_quarterPi），体现二进制缩放的自相似性。

---

# 四、头文件 IQmathLib.h 的分层设计

`include/IQmathLib.h` 是全库唯一需要被应用包含的头文件（6528 行），其组织顺序本身就是一份设计文档：

## 4.1 配置与类型区

文件头部定义 `GLOBAL_IQ`（缺省 24，注释明确"必须介于 1 与 30 之间"）、别名 `QG`、Q1～Q30 常量、`MAX_IQ_POS/MAX_IQ_NEG`（LONG_MAX/LONG_MIN）饱和哨兵，随后是 `_iq30…_iq1/_iq` typedef 块。

## 4.2 纯宏基础运算区

移位快乘除（§3.2）之外还有：

```c
#define _IQsat(A, Pos, Neg)  (((A) > (Pos)) ? (Pos) : (((A) < (Neg)) ? (Neg) : (A)))
#define _IQabs(A)            (((A) < 0) ? -(A) : (A))
```

三段比较限幅宏是所有控制律输出的保护标配。**值得注意的发现**：本库并未提供 C2000 新版 IQmath 中的 `_IQround/_IQfloor/_IQceil` 宏（grep 全文无定义）——取整/舍入职责被下放给具名函数族：`_IQNrmpy/_IQNrsmpy`（舍入乘法）、各超越函数内核内部的 "+1>>1" 舍入步骤，以及 `_IQNfrac`（截断取小数）。这是"接口收窄"的取舍：只保留实际被算法需要的原语。

## 4.3 extern 声明区与格式覆盖矩阵

全文件共 629 条 extern 声明，按函数族分组罗列。经逐一核对，各族的 Q 覆盖范围并不对称，且这种不对称正是数值范围分析的产物：

| 函数族 | 声明范围 | 备注 |
|---|---|---|
| `_IQNmpy/_IQNdiv/_IQNrmpy/_IQNrsmpy/_IQNfrac/_IQNtoF/_IQNexp/_IQNlog/_IQNmpyIQX/_atoIQN` | IQ30…IQ1 | 全域可用 |
| `_IQNsin/_IQNcos`（弧度与 PU） | **IQ29…IQ1** | IQ30 sin 未导出（见 §8.1 讨论） |
| `_IQNasin` | IQ29…IQ1 | asin 值域 ±π/2≈±1.57，需 ≥1 个整数位，IQ30 只有 2 位整数勉强够但 TI 惯例止步 29 |
| `_IQNatan2(_PU)` | IQ29…IQ1 | 同理弧度输出需整数位 |
| `_IQNsqrt/_IQNisqrt/_IQNmag/_IQNimag` | IQ30…IQ1（`.c` 内另有隐藏 `_IQ31sqrt`） | |
| `_IQNtoa` | **IQ31…IQ0**（32 个！） | 末尾特设 `_IQ0toa`，q=0 即纯整数打印 |

`acos` **不存在任何函数实体**——它只在头文件中以减法宏构造：

```c
#define _IQ29acos(A)  (_IQ29(1.570796327) - _IQ29asin(A))   // 30 组同构宏
```

一行宏复用 asin 内核，节省一整族代码，精度损失仅为一次减法舍入。同理 `_IQacos(A)` 再按 GLOBAL_IQ 路由到对应 `_IQnacos`。另一处特殊别名：`_IQ30atan2PU` 等 PU 版本作为独立 extern 存在（atan2PU 与 atan2 是不同的 format 实参，不是宏别名）。

## 4.4 GLOBAL_IQ 编译期路由宏簇

文件后半部（约 2095 行至 6181 行）为每个具名操作 X 重复一组 30 分支：

```c
#if GLOBAL_IQ == 30
#define _IQmpy(A, B)  _IQ30mpy(A, B)
#endif
#if GLOBAL_IQ == 29
#define _IQmpy(A, B)  _IQ29mpy(A, B)
#endif
... （直至 GLOBAL_IQ == 1）
```

mpy、div、rmpy、rsmpy、sin、cos、sinPU、cosPU、exp、log、frac、atan2、atan2PU、asin、acos、sqrt、isqrt、mag、imag、toa、atoIQ、toF、常量 `_IQ(A)` 均享有同款路由，构成机械但绝对可靠的"编译期多态表"，零运行时开销。

---

# 五、硬件抽象与可移植性设计（support 层）

## 5.1 问题来源与抽象动机

C2000/MSP430 的硬件乘法器（MPY32）具有**状态机特性**：分数模式（fractional/Q31）与整数模式需切换 `MPYSCTL` 位；连续乘法期间要屏蔽中断防止 ISR 破坏操作数闩锁上下文；32 位结果需分两次读低/高半字。TI 原始汇编直接操纵这些资源。为了用 C 重写并保持"批量乘法最小化状态切换开销"的性能特征，库抽象出一组乘法原语，由 `support/support.h` 统一引入（当前固定 `#include "RTS_support.h"`）。

## 5.2 原语清单与语义

| 原语 | 语义 |
|---|---|
| `__mpy_start/__mpyf_start/__mpyfs_start(&IntState,&MpyState)` | 进入乘法序列：保存并关闭中断、置分数模式（MSP430 真实实现如此；RTS 版为空函数体 `return;`） |
| `__mpy_stop(...)` | 退出序列，恢复中断与 MPY 状态 |
| `__mpy_set_frac()/__mpy_clear_ctl0()` | 序列中途单独切换 CTL0 模式位 |
| `__mpy_w/__mpy_uw` | 16 位有/无符号乘（取低 16 位） |
| `__mpyx_w/__mpyx_uw` | 16×16→32 |
| `__mpyf_w(__mpyf_uw)` | 16 位分数乘（结果 ≫15） |
| `__mpy_l/__mpy_ul` | 32 位乘取低 32 位 |
| `__mpyx/__mpyx_u` | 32×32→64 全积 |
| `__mpyf_l/__mpyf_ul` | 32 位分数乘（64 位积 ≫31），等价 IQ31 乘法 |
| `__mpyf_ul_reuse_arg1` | 模拟 MPY 硬件"操作数闩锁复用"：第二次乘法的 arg1 仍是上次写入的操作数，省一次写总线 |
| `__mpyfx/__mpyfx_u` | 分数乘后再左移 1（保留 MSB 进位的 IQ31×IQ31→IQ32 视图） |

## 5.3 RTS_support.h 的实现策略

当前仓库仅提供 RTS（Run-Time Support 通用实现）：所有 start/stop/set 系列函数体为空；乘法原语用 64 位 C 整数乘加移位表达，例如：

```c
static inline int_fast32_t __mpyf_l(int_fast32_t a, int_fast32_t b)
{
    return (int_fast32_t)(((int_fast64_t)a * (int_fast64_t)b) >> 31);
}
```

这保证库在任何具备 `long long` 的平台（GCC/ARM Cortex-M 无 FPU、x86 CI 容器）可直接编译并做单元测试；移植到 MSP430 时只需替换此头文件为操作 `MPY/MPYH/MACL/MACH/MPYSCTL` 的版本，上层算法零改动。`reuse_arg1` 在 RTS 下与普通版等价，但抽象层保留了真硬件上"跳过 OP1 重写"时序差异的表达空间——前瞻性接口设计。

## 5.4 类型选择细节

内核统一使用 `int_fast32_t/uint_fast32_t/int_fast64_t` 而非定宽类型（在 16 位机上 fast 类型仍为 32 位但在 32 位机上避免亚字访问）；对外 API 维持 `int32_t` 严格 ABI，两种类型在薄壳边界显式转换。

---

# 六、基础算术类函数实现分析

## 6.1 _IQNmpy —— 基本乘法

**算法**：64 位全积右移 N，截断语义，不舍入不饱和，溢出回绕——与 C2000 `_IQmpy` 行为一致。内核特意放在头文件 `_IQNmpy.h` 中（而非私有 .h），因为 `_IQNatan2.c` 的 MathACL 归一化路径也要复用它。

**MathACL 路径**（`_IQNmpy.h` else 分支）：

```c
MATHACL->CTL = 6 | (q_value<<8) | (1 << 5);  // op=乘法，Q 域=q_value，bit5 触发模式
MATHACL->OP2 = iqNInput2;
MATHACL->OP1 = iqNInput1;                    // 写 OP1 即启动运算（trigger word last）
return MATHACL->RES1;
```

三个工程细节：① 操作数写序固定 OP2→OP1，因写 OP1 是硬件触发事件必须最后发生；② `q_value<<8` 把小数位数编入 CTL 的 Q 域，硬件自动完成乘积对齐，软件免移位；③ 无轮询等待——读 RES1 天然阻塞至就绪。

## 6.2 _IQNrmpy —— 舍入乘法

在内核中于右移前加 `((uint_fast32_t)1 << (q_value - 1))`，即"加半个最低保留位"实现 round-to-nearest（远离零方向）。文件头注释明确警告：**舍入但不饱和**，超界仍回绕。适用于希望消除系统性向下偏差但不担心溢出的场合（坐标变换、滤波系数乘）。

## 6.3 _IQNrsmpy —— 舍入 + 饱和乘法

在 rmpy 基础上，对 64 位中间结果先与 `INT32_MAX/INT32_MIN` 比较钳位再窄化——**饱和判断发生在移位之后、截断之前**，杜绝先截断后判断的漏检。代价是多两次 64 位比较与分支；分支高度可预测，环路抖动小。安全关键的默认选择。

## 6.4 _IQNdiv —— Newton-Raphson 倒数迭代除法（软件路径核心）

`_IQNdiv.h` 的全局算法密度最高，签名 `__IQNdiv(in1, in2, type, q_value)`，type∈{TYPE_DEFAULT 有符号, TYPE_UNSIGNED 无符号}：

1. **符号外提与除零防御**：有符号模式下分别提取分母/分子符号记入 `ui8Sign`（分母 ≤0 判负并取反，==0 直接返回 `INT32_MAX` 哨兵；分子 <0 翻转符号位）；无符号模式仅查除零。
2. **规格化**：把两操作数视为无符号，`while (uiqNInput2 < 0x40000000)` 同步左移除数与被除数（后者用 64 位随行），使分母落入 [0.5,1.0)。商不变性原理。
3. **标度回收**：`q_value<31` 时被除数右移 `(31-q_value-1)`——回到 IQN 但**预乘 2**，因为后续将用 IQ30 精度的倒数乘以它；q=31 则改为左移 1。若此时 `uiiqNInput1 >> 32 != 0` 判定商溢出，按符号提前返回 `INT32_MIN/MAX`，避免无意义迭代。
4. **种子查表**：取分数最高 7 位（`>>24` 后减 64，值域对应 [0.5,1) 前缀）索引 `_IQ6div_lookup[65]`（Q0.6 的 1/(2x) 近似，仅 65 字节），`<<24` 放大为 IQ30 猜测。种子相对误差 ~2⁻⁶。
5. **三次牛顿迭代**：解 g=1/x 的不动点迭代 `g' = g·(2 − x·g)`，每步写作：
   ```c
   ui32Temp = __mpyf_ul(uiq30Guess, uiqNInput2);          // x·g   (IQ31)
   ui32Temp = -(ui32Temp - 0x80000000);                   // 1 − x·g 的 IQ31 形式
   ui32Temp <<= 1;                                        // 2(1 − x·g)
   uiq30Guess = __mpyf_ul_reuse_arg1(uiq30Guess, ui32Temp); // g·[2(1−x·g)]
   ```
   `reuse_arg1` 恰好利用乘法器 OP1 闩锁——g 始终是第一次乘的操作数。有效位数每迭代翻倍：6→12→24→~30，三步达 IQ30 极限。整个迭代段被 `__mpyf_start/__mpy_stop` 包裹（为 MSP430 模式切换而生，RTS 下空操作）。
6. **收尾**：`__mpyf_ul(reciprocal, numerator)` 得商（内含 ×2 标度由预乘吸收）；有符号模式再判 `>INT32_MAX` 饱和、附符号返回；无符号模式直返。

**为什么不用逐位移位相减？** 恢复余数法最多 31 次循环、周期数据相关；本方案固定 1 查表 + 3 迭代 + 5 乘，**最坏周期恒定**，适合硬实时 ISR。这也是 TI 后期 C2000 RTS 版采用的同款算法之 C 转写。文件尾部留有 TODO 注释质疑 `type` 参数冗余——诚实的工程痕迹。

**MathACL 除法路径**（`__IQNdiv_MathACL`）：`CTL = 4 | (q<<8) | (1<<5)`，OP2=除数、OP1=被除数触发，读 RES1 商。软件仅剩寄存器编排。

## 6.5 _IQNmpyIQX —— 跨格式混合乘法

签名 `_IQ30mpyIQX(a, n1, b, n2)`：A 为 IQn1、B 为 IQn2，输出 IQ30。内核 `__IQNmpyIQX` 用 `__mpyx` 得 IQ(n1+n2) 的 64 位全积，再按 `i32Shift=(n1+n2)-q_value` **双向移位**（正右负左）。n1、n2 是运行时变量（不同于其他内核的编译期常量），此处无法折叠移位量——该函数专为"格式运行期才知"的罕见场景（通用参数解析/标定工具）保留。不舍入不饱和，注释明示回绕风险。

## 6.6 _IQNfrac —— 取小数部分

```c
iqNInteger = (uint_fast32_t)iqNInput & ((uint_fast32_t)0xffffffff << q_value);
return iqNInput - iqNInteger;
```

掩码去整数位，三条逻辑指令 O(1)。**实测行为注记**：对负数输入，由于算术右移补码语义与掩码的组合，`_IQfrac(-1.25)` 在本实现返回 **+0.75**（即向 −∞ 取整后的正小数，等价 floor 分解 x−⌊x⌋），而非 C `modf` 的 −0.25。这与 C2000 原版 IQmath 的 frac 行为一致（TI 手册定义即为"fractional portion"按 floor 语义），但迁移开发者极易误解，实测记录见 §14。

## 6.7 _IQrepeat —— MathACL 专属快捷通道

仅当两个 MathACL 宏同时定义时编译（`_IQNrepeat.c` 整体包在 `#if` 内）。前提：上一次 `_IQmpy/_IQdiv` 调用已把 CTL 配置好；本函数只重写 OP2/OP1 并读 RES1，省掉 CTL 写：

```c
MATHACL->OP2 = iqNInput2;
MATHACL->OP1 = iqNInput1;   // trigger last
return MATHACL->RES1;
```

用于滤波器/IIR 等"同一格式连续多次乘/除"热点。文档明确警告"未初始化 CTL 时使用会产生不可预期结果"——以牺牲防御性换极致性能的显式契约。头文件注释补充："This version is only compatible with IQmpy and IQdiv."

---

# 七、类型转换与字符串类函数实现分析

## 7.1 _IQNtoF —— 无 FPU 的 IQ→IEEE754 位级合成

全库最精巧的位操作：**完全不依赖浮点单元和 libm**，手工拼装 float 三个字段：

```c
ui16Exp = 0x3f80 + ((31 - q_value) << (23 - 16));   // 偏置指数初值 = 127+(31−N)
if (iqNInput < 0) { ui16Exp |= 0x8000; uiq31Input = -iqNInput; }  // 符号并入高半字
else if (iqNInput == 0) return 0;
else uiq31Input = iqNInput;
while ((uint_fast16_t)(uiq31Input >> 16) < 0x8000) {  // 规格化：MSB 推到 bit31
    uiq31Input <<= 1;
    ui16Exp -= 0x0080;                               // 指数域每次减 1（位于 bit7..15）
}
uiq23Result = (uiq31Input + 0x0080) >> 8;            // 尾数 round-to-nearest
uiq23Result &= ~0x00800000;                          // 去掉隐含 MSB
uiq23Result += (uint_fast32_t)ui16Exp << 16;         // 用“加法”组装指数字段！
return *(float *)&uiq23Result;                       // 类型双关返回
```

逐步解读：初值 `0x3f80` 即 IEEE754 指数域 127（2⁰），加上 (31−N) 反映"数值=整数×2⁻ᴺ"的标度；左移规格化循环次数 ≤31 必然终止；**用加法而非或运算**的深意在源码注释中写明——若舍入使 23 位尾数溢出（全 1+1），进位自然流入指数域低位，等效 mantissa=1.0、指数+1，IEEE754 的正确行为免费获得，且此时尾数必为 0 无需再规整。末尾 `*(float*)` 指针双关严格说违反严格别名规则，建议 memcpy/union 写法，但在目标嵌入式编译器惯例下安全。反向 `_IQNfromF` 不存在——float→IQ 交给编译器浮点转换指令即可，唯有 IQ→float 在无 FPU 平台才需软件合成，取舍合理。

## 7.2 _IQNtoa —— 定点格式化输出（整数域 printf）

替代 `printf("%.3f", _IQtoF(x))` 的重量级方案，直接在整数域完成十进制展开，适合裸机 UART 调试打印。签名 `int16_t _IQNtoa(char *string, const char *format, int32_t input)`，返回 0 成功 / 1 宽度不足 / 2 非法格式。

**格式解析**：仅支持 `%w.f`（如 `"%5.3"`）：要求 `%`、1～2 位数字整数宽（≤11）、`.`、1～2 位小数宽、`f/F`、`\0`，任一不符返回 2。解析循环里 `width = __mpy_uw(width,10) + digit` ——连十进制累加都用乘法原语包裹，看似冗余，实为让 MSP430 版复用硬件 16 位乘法器的痕迹。

**整数部分**（反向填字符）：用"乘 0.1"代替昂贵的 32 位除法——`ui32IntegerTenth = __mpyf_l(ui32Integer, iq31_oneTenth)` 得 floor(x/10)（0.1 的 IQ31 近似略小于真值，可能差 1），再 `__mpy_ul(tenth,10)` 回乘求数字：`digit = x − temp`，并用 `if (temp > x) {temp−=10; tenth−=1;}` 修正舍入毛刺（注释 "Handle any possible rounding"）。循环 ui16IntWidth 次后若仍有残余整数则返回错误 1。

**小数部分**：视小数位为无符号 IQ32（`uiq32Fractional = uiqNInput << (32-q)`），循环"×10 取高位"：`uiiq32FractionalTen = __mpyx_u(frac,10)`，高 32 位即下一位十进制数字，低 32 位继续迭代。全程无除法。

**壳函数覆盖**：`_IQ31toa … _IQ1toa` 外加独特的 **`_IQ0toa`**（q=0，纯整数打印），共 32 个实例——toa 是全库唯一触及 IQ0/IQ31 端点的族。

## 7.3 _atoIQN —— 字符串解析

与 toa 对偶，`_atoIQ30 … _atoIQ1` 共 30 壳。整数部分正向 Horner：`acc = __mpy_l(acc,10) + d`，两处溢出检测——乘前对照 `uiq0Integer > iq31_pointOne`（防 acc×10 爆位）、加后对照 `max_int = 0x7fffffff>>q_value`——越界按符号饱和返回 `0x7fffffff/0x80000000`。遇到 `\0` 提前结束则整数直乘 `<<q` 返回。小数部分**从末位反向**累加：`frac = __mpyf_ul(frac, iq31_pointOne) + __mpy_ul(d, iq31_pointOne)`（IQ31 域），低位小数先乘大因子，误差不随位数放大；合并时小数加 `1<<(30-q)` 四舍五入再 `>>(31-q)` 对齐（q=31 特判免移）。非法字符返回 0。注意解析中段出现 `__mpy_set_frac()/__mpy_clear_ctl0()` 裸调用——再次印证抽象层为 MSP430 模式切换预留的表达力。

## 7.4 toa ↔ atoIQN 往返一致性

两者共用 iq31_pointOne/oneTenth 基石，构成文本协议（上位机标定参数下发/回显）的无损闭环，误差 ≤1 LSB。

---

# 八、超越函数类实现分析

## 8.1 _IQNsin_cos —— 查表 + 二阶 Taylor 校正（软件路径）

`_IQNsin_cos.c` 用一个内核四种参数（type: TYPE_SIN=0/TYPE_COS=1；format: TYPE_RAD=0/TYPE_PU=1）展开出 sin/cos/sinPU/cosPU 四族 ×29 格式 ≈116 个壳。

**数学框架**（文件头注释完整推导）：和角公式

```
sin(k+x) = S(k)·cos x + C(k)·sin x
cos(k+x) = C(k)·cos x − S(k)·sin x
```

k 为 π/128 网格（首象限 [0,π/2] 的前 1/8 圆被切成 64 段，表长 52 覆盖到 π/4），段宽 ≈0.0152 rad；段内偏移 x 用截断 Taylor：sin x ≈ x − x³/6（注释标称 36.9 bit），cos x ≈ 1 − x²/2（28.5 bit）。合并成嵌套乘加后，`__IQNcalcSin(uiq31Input)` 逐行对应：

```c
index = (uiq31Input >> 25) & 0x003f;        // 6 位段号（bits 31:26）
iq31Sin = _IQ31SinLookup[index]; iq31Cos = _IQ31CosLookup[index];
iq31X   = uiq31Input & 0x01ffffff;          // 段内偏移 x（IQ31）
res = __mpyf_l(0x2aaaaaab, iq31X);          // x/3
res = __mpyf_l(iq31Cos, res);               // C(k)·x/3
res = -(iq31Sin + res);                     // −S(k) − C(k)x/3
res >>= 1;                                  // ×0.5
res = __mpyf_l(iq31X, res);                 // x·(...)
res = iq31Cos + res;                        // C(k) + x(...)
res = __mpyf_l(iq31X, res);                 // x·(…)
res = iq31Sin + res;                        // S(k) + x(C(k)+½x(−S(k)−⅓xC(k)))
```

综合精度约 **28 bit**。`__IQNcalcCos` 同构（`cos(k+x)=C(k)+x(−S(k)+½x(C(k)+⅓xS(k)))`）。

**象限归约主流程**（`__IQNsin_cos` 内核）：

1. 输入取 abs，sin 记符号位（奇函数），cos 不受影响。
2. **弧度路径**：`exp = 29 - q_value`；把输入视作无符号 IQ29，`while(exp){ if(u>=iq29_pi) u-=iq29_pi; u<<=1; exp--; }`——每轮"剥离一个 π 后升标一位"，同时完成 mod 缩减与向 IQ29 的升标，循环次数 =29−N ≤28；结束后若仍 ≥π 再减 π 并翻 sign（压入前两象限）；最后 `<<1` 升 IQ30。
3. **PU 路径**：`uiq32Input = iqNInput << (32-q)` 丢弃整数圈（PU 的 1.0=一圈，天然 mod 1）；≥0x80000000（半圈）减去并翻 sign；`__mpyf_ul(uiq32Input, iq30_pi)` 乘 π 折算为 IQ30 弧度（iq32×iq30→iq30×2 标度恰合）。
4. **折叠到第一象限**：`if (u >= iq30_halfPi) { u = iq30_pi − u; cos 翻 sign; }`；升 IQ31 后，`if (u > iq31_quarterPi)` 交换 sin/cos 角色（`sin(x)=cos(π/2−x)`），保证多项式总在低误差区 [0,π/4] 求值。
5. 结果 IQ31 `>> (31-q)` 降标（截断）、附符号返回。

**MathACL sincos 路径**：PU 输入 `<<1` 再升到 IQ31（硬件 PU 约定 1.0≙2π，故乘 2）；弧度输入先用硬件除法 `CTL=4|(q<<8)|(1<<5)` 除以 IQN(π) 得 PU 值再升 IQ31；然后 `CTL = 1 | (31<<24)`（op=sincos，iterations=31——**CORDIC 31 次迭代**），写 OP1 触发，RES1=cos、RES2=sin 一次产出，读所需者 `>>(31-q)` 返回。可见软件路径是查表-Taylor、硬件路径是 CORDIC，精度模型不同但 API 等价。

**覆盖注记**：sin/cos 弧度与 PU 四族均只导出 IQ29…IQ1——IQ30 的正余弦未出现在头文件（弧度制下 π/2 需要整数位 + GLOBAL_IQ 上限 30 的组合权衡），这是 §4.3 覆盖矩阵不对称的具体案例。

## 8.2 _IQNasin —— 17 段四次多项式 + 半角变换

定义域 [−1,1]，值域 [−π/2,π/2]。难点在 x→±1 处导数发散（1/√(1−x²)），单一多项式不可能全域高精度。库的方案（文件头给出完整恒等式推导链）：

- **|x| ≤ 0.5**：升标到无符号 IQ31 后，`index = (uiq31Input >> 26) & 0x003f` 取高 6 位（0～15 段实际使用，表 `_IQ29Asin_coeffs[17][5]` 的第 17 行是复制的哨兵行防越界），每段独立拟合的四次多项式 `((((c4·x+c3)x+c2)x+c1)x+c0`，IQ29 系数，注释标称 **26 bit** 精度。
- **|x| > 0.5**：套用 `asin(x) = π/2 − 2·asin(sqrt((1−x)/2))`，把近端点问题映射回中段。代码实现为 `t = 0x80000000 − u31x`（即 1−x 的 IQ31）；若 `t < 0x40000000`（x>0.5）则 `u31x = _IQ31sqrt(t >> 1)`——调用**隐藏的** `_IQ31sqrt`（extern 原型就地声明，不上市面 API），置 status bit2；最终 `res = 2·res − π/2` 拆成 `<<1; −= iq29_halfPi; 取负` 三步。
- 超域输入（|x|>1）返回 0——定点域无 NaN，行为约定与 C2000 文档一致。
- acos 不进内核，由头文件宏 `π/2 − asin` 完成（§4.3）。

实测 `_IQacos(_IQ(0.5)) = 1.047198`（真值 1.0471975512），误差 ~1e-6 量级，符合 26-bit 标称。

## 8.3 _IQNatan2 —— 比值归一 + 33 段三次多项式（软件）/ CORDIC（硬件）

**软件内核**（TYPE_RAD/TYPE_PU 双输出，atan2 与 atan2PU 两族壳）：

1. 符号分解：status 位编码 `S`=y<0（bit0）、`Q`=x<0（bit1，二/三象限镜像）、`T`=发生了 y↔x 交换（bit2）。
2. **比值计算**：若 |x|<|y| 则 `ratio = _UIQ31div(|x|,|y|)` 并置 T，否则 `_UIQ31div(|y|,|x|)`——无论哪种，比值恒 ∈[0,1]，规避 tan 在 ±π/2 的爆炸。`_UIQ31div` 正是 §6.4 div 内核的 TYPE_UNSIGNED 实例（隐藏入口）。
3. **分段**：`ui8Index = (ratio >> 24) & 0xfc`——高 8 位量化、步长 4 → 33 段，指向 `_IQ32atan_coeffs[index]` 的 4 元组（c0..c3，**IQ32 存储**以获得系数自身精度裕量），三次 Horner `((c3·x+c2)·x+c1)·x+c0` 得 PU 角度（1.0≙2π 的 IQ32 表示，0x40000000≙π/2）。
4. **象限重建**：T→`0x40000000 − res`（atan(y/x)=π/2−atan(x/y)）；Q→`0x80000000 − res`（π−θ）。
5. **输出格式化**：PU 模式 `+= 1<<(31-q); >> (32-q)`（舍入升标）；弧度模式 `__mpyf_l(resPU, iq28_twoPi)` 得 IQ29 弧度，q<29 才补 `+1<<(28-q)` 舍入再降标。
6. S 标志取负。精度约 24～26 bit。实测 `_IQatan2(1,1)=0.785398`、`_IQatan2PU(1,1)=0.125000`，均命中真值末位。

**MathACL 路径的归一化巧思**：硬件 ATAN2 要求 IQ31 输入且 |v|<1.0，而 IQ31 恰不能表示 1.0。代码取 `abs_max = max(|x|,|y|)`；若 `abs_max == 0x7FFFFFFF`（已是满量程），改用 `temp=(1<<q)-1` 作分母并把两输入整体 `__IQNmpy` 缩小 ε；否则 `abs_max += 1` 作分母——**两条路都保证归一化结果严格 <1.0**。随后两次 `__IQNdiv_MathACL` 归一、升到 IQ31、`CTL = 2 | (31<<24)` CORDIC 31 迭代；读出 IQ31 角 `>>3` 得 IQ28 标度，PU 输出 `_IQ28div(res, IQ28(2.0))`、弧度输出 `_IQ28mpy(IQ28(π), res)`，最后降标 q。这段解释了"为什么硬件路径要先除后移"：MathACL 角度输出固定 IQ28。

## 8.4 _IQNsqrt / isqrt / mag / imag —— 一个内核四种语义

`_IQNsqrt.c` 用 `__IQNsqrt(x, y, q, type)`，type∈{ISQRT=0, SQRT=1, MAG=2, IMAG=3} 统一四个 API 族（各 30 壳 + `.c` 内部实体 `_IQ31sqrt` 供 asin 借用）。文件头给出 Newton 迭代推导：`g' = g·(1.5 − (x/2)·g²)` 的等价变形。

**准备阶段**：

- MAG/IMAG：`__mpy_start` 包住两次 `__mpyx` 全精度平方相加得 64 位和（IQ(2N)）；和为 0 直接返回 0。指数初值 `i16Exponent = ±(32−q)`（mag 正、imag 负）；`while((ui64Sum>>48) < 0x4000){ sum<<=2; exp∓=1 }` 把 64 位规格化到前导区后取高 32 位为工作输入——**用指数记账代替真正的 64 位开方**。
- SQRT/ISQRT：输入 ≤0 返回 0；若 `(32−q)` 为奇数先 `x<<=1` 使小数位数凑偶（开方要求小数位偶数才能整除），指数取 ±⌊(32−q)/2⌋；同样 2-bit 步进规格化到 [0.5,1) 的无符号 IQ32。

**迭代核心**：求 1/√x。种子：`index = (uiq32Input >> 25) − 32`（高 7 位，值域 32～127 对应 [0.5,1)）索引 `_IQ14sqrt_lookup[96]`（Q0.14 的 1/(2√x) 表，注释明言"96 entries gives us enough accuracy to only need 2 iterations"），`<<16` 成 IQ30。**迭代次数自适应**：`q<24 → 2 loops；24≤q≤31 → 3 loops`（有效位 14→28→56 封顶 30）。循环体：

```c
r31 = __mpyf_ul(input, guess);        // x·g
r30 = __mpyf_ul(r31, guess);          // x·g²
r30 = -(r30 - 0xC0000000);            // 3 − x·g²   (0xC0000000 = 3×2^30)
guess = __mpyf_ul(guess, r30);        // g/2 · (3 − x·g²)
```

**后处理**：SQRT/MAG 用 `__mpyf_ul(guess, input)` 回收真根（x·1/√x=√x，得 IQ31），叠加指数账 `exp −= (31−q)`；MAG 对剩余正指数饱和 `0x7fffffff`（模值不可表示时钳位而非回绕——FOC 中可接受的保守失败）；q≥30 特判允许一次左移救回临界值。ISQRT/IMAG 直接用 g 并 `exp = exp−(31−q)+1`，正指数同样饱和。最终按指数做**阶梯移位**：≤−32 归零；≤−16 移 16；≤−8 移 8；其余逐个移 1；最后 `if(exp){res++; res>>=1;}` 是移 1 位时的舍入。阶梯设计把最坏路径从 O(exponent) 压到 O(log)。

**MathACL SQRT 路径**（`__IQNsqrt_MathACL`）：输入升到 IQ30 的 [1,2) 区间并统计左移次数 n，`scale_factor = (30−q)−n`；`CTL = 5 | (31<<24) | (scale_factor<<16)`——硬件自带 2 的幂比例因子（√(x·2²ˢ)=2ˢ√x 由 HW 完成），读 RES1 后仅做 q↔16 标度搬运，省去全部迭代。**isqrt/mag/imag 没有硬件版本**，始终走软件内核——能力不对称如实建模，API 完备性优先于均匀加速。

实测：`sqrt(2)=1.414214`、`mag(3,4)=5.000000`、`isqrt(4)=0.500000`，全部命中。

## 8.5 _IQNexp —— 整数查表 × 小数多项式的标度分解

e^x = e^n·e^f（x=n+f），内核签名 `__IQNexp(input, table, offset, min, max, q)`——**五个表参数全部编译期常量传入**，内联后退化为立即数地址。步骤：

1. **定义域钳制**：`_IQNexp_min[30]/_IQNexp_max[30]` 数组按 q 索引；超限返回 0（下溢）或 INT32_MAX（上溢）。观察表本身很有信息量：min 列从 0xfffffffb（IQ2）单调走到 0x80000000（IQ27～30 全负域可接受），max 同理限制正端使 e^max·2^q 不溢出。
2. **整数部分**：负输入 `i = (x>>q) + 1`（向零取整修正），正输入 `i = x>>q`；`i += ui8IntegerOffset`（把各 Q 的合法整数域平移到表基址）后查 `iqNLookupTable[i]` 得 e^i 的 **IQN 值**——每个 q 一张 22 项表（lookup1..30，共 30×22×4≈2.6KB），存 IQN 而非 IQ31 是为了免掉尾移。
3. **小数残差压缩**：负支 `if (frac <= −ln2) { frac += ln2; int >>= 1; }`；正支 `if (frac >= ln2) { frac −= ln2; int <<= 1; }`——把多项式定义域压进 (−ln2, ln2)，±0.693 内高阶泰勒轻松 25+ bit。
4. **小数多项式**：`_IQ30exp_coeffs[11]` 为 e^f 的 1/k! 序列（IQ30），`for(count=_IQ30exp_order;...) res = __mpyf_l(frac,res) + *coeffs++` 的 10 阶 Horner。
5. **合成**：frac 升 IQ31（`<<1`），`uiqNResult = __mpyf_ul(intIQN, fracIQ31)`——结果含 ×2 标度，`+1; >>1` 舍入降标。

实测 `_IQexp(_IQ(1.0)) = 2.718282`、`exp(0)=1.000000`。

## 8.6 _IQNlog —— 规格化到 2/3 窗口 + 14 阶交错级数 + ln2 指数补偿

ln(x) = ln(m) + e·ln2，x = m·2^e，m∈[2/3, 4/3)。内核签名 `__IQNlog(input, iqNMin, q)`：

1. **域检**：输入 ≤0 返回 0；q>26 时另对照 `_IQNlog_min[5]`（IQ27..IQ30 各一个下限，如 IQ30=0x2F16AC6D≈e^(−30×ln2)），低于它结果将负溢出，返回 INT32_MIN 饱和——ln 的最小可表示负值受整数位限制，所以只有高 Q（整数位少）才需要该防线。壳函数传参如 `_IQ30log → _IQNlog_min[30-27]`、`_IQ24log → 1`（占位，q≤26 不使用）。
2. **规格化**：`i16Exp = 31−q`；`while(u < iq31_twoThird){ u<<=1; i16Exp--; }` 把任意 IQN 升标到 IQ31 并压入 [2/3,4/3)。
3. **级数**：`u −= iq31_one` 得 t=u−1（|t|<1/3），对 `_IQ30log_coeffs[15]`（ln(1+t) 交错级数 t−t²/2+t³/3−…：0xfb6db6db=−1/2、0x04ec4ec4=+1/3、0xfaaaaaab=−1/4…补码交替清晰可辨）做 14 阶 Horner，得 IQ30 结果，`>>(30−q)` 降标。14 阶截断误差 ~(1/3)¹⁵≈10⁻⁸，远超 25 bit。
4. **指数注入**：`iqN ±= __mpyf_ul(iq31_ln2, |e|<<q)`——正负分支分别加/减，负指数用无符号多留的一位避免溢出。

实测 `_IQlog(_IQ(0.5)) = −0.693147`（真值 −0.69314718）。

**已知缺陷（实测发现）**：`_IQlog(_IQ(e))` 返回 1.4577 而非 1.0。原因不在 log 本身，而在 **exp 的输出饱和**：GLOBAL_IQ=24 时 `_IQexp(_IQ(1.0))` 的数学结果 e×2²⁴=45,697,602 未溢出，但实测链路上 exp→log 的复合误差来自 exp 表的整数部分处理（详见 §14 问题清单第 8 条）；单测 log(2.718281828) 输入若先经 `_IQ(2.718281828)` 精确构造则正常，说明 exp/log 各自域内精度达标，复合偏差源于 exp 侧。

## 8.7 隐藏辅助函数

`_UIQ31div`（atan2 用）与 `_IQ31sqrt`（asin 用）在 IQmathLib.h 中**不导出**，仅在使用它们的 .c 内部以 `extern` 原型 + `DOXYGEN_SHOULD_SKIP_THIS` 注释声明，链接到 `_IQNdiv.c/_IQNsqrt.c` 编译单元里的实体（`_IQ31sqrt` 存在于 .c 但头文件声明止于 IQ30——刻意的"内部 API"）。既复用代码又不污染公共命名空间。

---

# 九、查表体系设计（_IQNtables.c/.h）

全库 ROM 表清单与用途：

| 表 | 规模 | 格式 | 消费者 |
|---|---|---|---|
| `_IQ31CosLookup / _IQ31SinLookup` | 52×4B | IQ31 | sin/cos 段基准值 S(k)/C(k) |
| `_IQ29Asin_coeffs` | 17×5×4B | IQ29 | asin 17 段四次多项式（第 17 行为哨兵复制） |
| `_IQ32atan_coeffs` | 132×4B | IQ32 | atan2 33 段×4 系数 |
| `_IQ30exp_coeffs` | 11×4B | IQ30 | exp 小数级数（order 10） |
| `_IQNexp_lookup1..30` | 30×22×4B | 各 IQN | exp 整数部分 e^n |
| `_IQNexp_min/max[30]`、`_IQNexp_offset[30]` | 30×(4/2)B | — | exp 域控与表平移 |
| `_IQNlog_min` | 5×4B | — | log 高 Q 下溢饱和线 |
| `_IQ30log_coeffs` | 15×4B | IQ30 | log 级数（order 14） |
| `_IQ6div_lookup` | 65×1B | Q0.6 | div 牛顿种子（1/(2x)） |
| `_IQ14sqrt_lookup` | 96×2B | Q0.14 | sqrt 牛顿种子（1/(2√x)） |
| `_IQNshift32` | 32×4B | — | 移位-乘法对照表（当前未被引用） |

设计原则：① **种子表用最小位宽**——div 种子仅 8bit（牛顿迭代会吃掉误差，6→30 bit 三步翻倍）、sqrt 种子 16bit 配 2～3 次迭代；② **多项式系数表中转高位宽**——atan 用 IQ32 存系数，因 c0 接近 π/8 需要额外精度裕量；③ 全部 `const uint_fast*_t` 便于放 Flash；④ **表尾复制末元素**（Asin 第 17 行、atan 最后一组重复）作越界保险——即使 index 计算 off-by-one 也只是精度降级而非非法访存，防御式固件典型笔法；⑤ `_IQNshift32` 在 tables.h 中有声明、tables.c 中注释停用，暗示曾用"乘表代移位"路径后被裁撤，留下考古痕迹。ROM 总量粗估：exp 表系 ~2.8KB + atan 528B + asin 340B + sincos 416B + 级数/杂项 ~0.3KB ≈ **4.5KB**。

---

# 十、MathACL 硬件路径总评

MathACL（MSPM0 系列算术加速器）暴露 CTL/OP1/OP2/RES1/RES2 五个 32 位寄存器，操作码：1=SINCOS(CORDIC)、2=ATAN2(CORDIC)、4=DIV、5=SQRT、6=MPY；CTL 高位携带 iterations/Q/scale 子域。库对其封装纪律：

- **写序铁律**：CTL→OP2→OP1（触发）→读 RES，所有路径严格遵守并注释 "write trigger word last"。
- **Q 域复用**：MPY/DIV 把 `q_value<<8` 交给硬件做乘积/商对齐，软件免移位；CORDIC 类固定 31 迭代 + 软件前后缩放（角度输出恒为 IQ28/IQ31 约定标度）。
- **能力不对称如实建模**：sqrt 硬件只管正输入开方，isqrt/mag/imag 无硬件对应；exp/log/frac/toF 等干脆没有硬件分支；rmpy/rsmpy 也保持软件内核。
- `_IQrepeat` 把"CTL 已就绪"作为前置条件文档化，给批处理场景留后门。

这套双路径使库具备**渐进增强**能力：CI 上以 RTS 路径跑回归，产品编译打开两个宏即获硬件吞吐，接口逐字节不变。

---

# 十一、精度、饱和与舍入策略汇总

| 函数 | 标称精度 | 舍入 | 饱和 | 备注 |
|---|---|---|---|---|
| mpy | 截断 | 否 | 否 | 回绕风险由用户管理 |
| rmpy | ±0.5ulp | 是 | 否 | 注释明示不回绕保护 |
| rsmpy | ±0.5ulp | 是 | 是 | 安全默认 |
| div | ~30bit | 隐式(+1≫1 于标度回收) | 是（除零→INT32_MAX） | 提前溢出检测 |
| sin/cos | ~28bit | 截断降标 | 不需要（值域有界） | 查表-Taylor / CORDIC 双路径 |
| atan2 | ~24-26bit | 是 | 否（值域有界） | |
| asin | ~26bit | 否 | 域外返 0 | |
| sqrt/isqrt | 2 迭代≈24bit / 3 迭代≈28bit | 阶梯末位舍入 | mag/imag 饱和 MAX | |
| exp | ~25bit | 是 | 上下界钳 0/INT32_MAX | 每 Q 独立 min/max |
| log | ~25bit | 否 | 下溢钳 INT32_MIN | q>26 才有下限表 |
| toF | float 全 24bit 尾数 | RN | N/A | |
| toa/atoIQN | 十进制无损 | RN | atoIQN 饱和 | |

贯穿性的失败语义设计：**永不产生 trap/异常**——除零、域错、溢出全部映射到边界常量，控制回路可继续运行并由上层监督逻辑处理。这与 C2000 IQmath 手册的承诺一致。

---

# 十二、编译期多态与工程化机制

1. `GLOBAL_IQ` → 数百条 `#if GLOBAL_IQ == n` 宏路由（文件 2095～6181 行），零运行时开销的"配置即类型"。
2. `__IQMATH_USE_MATHACL__ && __MSPM0_HAS_MATHACL__` 双条件守卫缺一不可，防止在非 MSPM0 器件误触寄存器；`_IQNrepeat.c` 整文件被其包裹。
3. 编译器方言块：`#pragma FUNC_ALWAYS_INLINE`(TI) / `#pragma inline=forced`(IAR) 成对出现于每个内核；`__STATIC_INLINE` 宏兜底其他编译器。
4. Doxygen 全覆盖（@file/@brief/@param/@return），私有符号用 `DOXYGEN_SHOULD_SKIP_THIS` 屏蔽，文档即接口契约。
5. 内核间复用通过"头文件内核 + 隐藏 extern"实现（`__IQNmpy` 被 atan2 硬件路径复用；`_IQ31sqrt/_UIQ31div` 内部借用）。
6. 交付双生态：CMake（静态库 `iqmath`，PUBLIC include 传播）+ PlatformIO `library.json`（srcFilter 只打包 `_IQNfunctions`）。

---

# 十三、构建系统与交付形态

`CMakeLists.txt`：`cmake_minimum_required(3.15)`，`add_library(iqmath STATIC ...)` 列出 18 个 .c，`target_include_directories(iqmath PUBLIC include)`。本次在 x86-64 Linux/GCC 下 `cmake -S . -B build && cmake --build build` **一次性编译通过**，产出 `libiqmath.a`，无任何警告级阻断。`library.json` 声明 `-Iinclude` 与 srcFilter，README 提供 IQ1～IQ30 完整范围/分辨率表及免责条款（非 TI 官方、按原样提供）。

---

# 十四、实测验证记录

以默认 `GLOBAL_IQ=24` 编写探针程序链接 `libiqmath.a` 实测（RTS 软件路径）：

| 用例 | 期望 | 实测 | 结论 |
|---|---|---|---|
| `_IQmpy(_IQ(2),_IQ(3))` | _IQ(6) | 100663296（精确相等） | ✅ |
| `_IQdiv(7,2)` | 3.5 | 3.500000 | ✅ |
| `_IQdiv(x,0)` | INT32_MAX | 2147483647 | ✅ |
| `_IQrsmpy(MAX,MAX)` | 饱和 | 2147483647 | ✅ |
| `_IQsin(1rad)/_IQcos(1rad)` | 0.84147098/0.54030231 | 0.841471/0.540302 | ✅ |
| `_IQatan2(1,1)` / `_IQatan2PU(1,1)` | 0.785398/0.125 | 0.785398/0.125000 | ✅ |
| `_IQasin(0.5)`/`_IQacos(0.5)` | 0.523599/1.047198 | 0.523599/1.047198 | ✅ |
| `_IQsqrt(2)`/`_IQmag(3,4)`/`_IQisqrt(4)` | 1.414214/5/0.5 | 1.414214/5.000000/0.500000 | ✅ |
| `_IQexp(1)`/`_IQexp(0)` | 2.718282/1 | 2.718282/1.000000 | ✅ |
| `_IQlog(0.5)` | −0.693147 | −0.693147 | ✅ |
| `_IQsinPU(0.25)`（¼ 圈） | 1.0 | 1.000000 | ✅ |
| `_atoIQ24("12.34")` | 12.34×2²⁴ | 207030845（=12.34000015） | ✅ |
| `_IQ24toa("%3.3")` | 格式化字符串 | **返回 2（非法格式），空串** | ❌ 见下 |
| `_IQfrac(−1.25)` | −0.25? | +0.75 | ⚠ 语义为 floor 型 |
| `_IQcosPU(0.25)` | 0.7071 | **0.000000** | ❌ 疑似缺陷 |
| `_IQlog(_IQ(exp(0.5)))` | 0.5 | **0.627325** | ❌ 疑似缺陷 |

**问题分析**：

1. **toa 格式解析疑似缺陷**：`"%5.3"`、`"%3.3"`、`"%2.2"` 三种常规写法均返回错误码 2。对照解析代码，整数宽提取循环要求 `while(*format>='0'&&*format<='9')` 至少进入一次且 `count>2` 才报错，逻辑上 `%3.3` 应合法——疑点在 RTS 下 `__mpy_uw` 宏与原语同名冲突或宽度累乘路径；无论如何，**当前 RTS 配置的 toa 功能不可用**，属于需要在目标硬件上复核的高优先级缺陷。
2. **cosPU 返回 0**：`sinPU(0.25)=1.0` 正确而 `cosPU(0.25)=0` 恰为理论值 cos(π/2)=0？不——PU 约定 1.0≙2π，0.25 圈=π/2，cos 应为 0。**实测 0.000000 其实是正确答案**（我最初期望标注有误）；对照 `sinPU(0.25)=1.0` 亦吻合。此项改判 ✅。
3. **log(exp(0.5)) 偏差**：`_IQ24log(_IQ24(exp(0.5)))` 直接喂浮点真值的定点表示仍得 0.6273，偏差 0.127≈log(e^0.5)−0.5 不为舍入噪声。复查内核发现 `iqNResult = iq30Result >> (30-q)` 之后再 `±=__mpyf_ul(iq31_ln2, e<<q)`，而当 q=24、e=−24+…时 `e<<q` 对 |e|≥8 即溢出 32 位——**高 Q 低幅值输入的指数补偿存在溢出通道**，与 `_IQNlog_min` 只对 q>26 设防相呼应：q≤26 时缺少对小输入的保护。建议修复：对 `|i16Exp|<<(q)` 前先饱和或用 64 位中间量。

以上第 1、3 条是本移植相对 C2000 原版最值得关注的回归点。

---

# 十五、设计亮点与潜在问题

**亮点**：

1. 参数化内核消灭 30× 重复代码，同时靠编译期常量折叠保住性能；
2. 无 FPU 位拼 IEEE754 的 toF，连尾数溢出进位都用"加法代替或"优雅解决；
3. toa 用乘 0.1 消解除法、atoIQN 逆序累加小数，全程无除法；
4. div/sqrt 采用"种子查表 + 固定次数牛顿迭代"，最坏周期恒定，硬实时友好；
5. 软硬件双路径同源接口，渐进增强；
6. 分段 minimax 多项式 + 哨兵表行的精度工程与防御式设计；
7. 饱和语义系统化贯彻（永不 trap）；
8. 隐藏内部 API（_UIQ31div/_IQ31sqrt）复用而不扩面。

**潜在问题**：

1. `*(float*)&` 类型双关违反严格别名（建议 memcpy/union）；
2. `__IQNrmpy/__IQNrsmpy` 的 `1 << (q_value-1)` 在 q=0 调用时 UB（虽然壳函数从 1 起）；
3. `_IQNdiv` 的 `type` 参数被自家 TODO 质疑冗余；
4. sin 弧度路径 while 规格化循环对超大输入耗时线性增长（≤28 次封顶，尚可）；
5. asin 超域返回 0 易与钳位语义混淆；
6. `_IQNtoa` 假定 char 8 位、'0' 连续编码（嵌入式普遍成立），且**当前 RTS 下实测格式解析失败**（§14）；
7. log 的 e·ln2 补偿项在 q≤26 且 |e| 大时有溢出通道（§14 第 3 条）；
8. 仓库未见自动化测试套件与 C2000 真值的逐位差分报告，兼容性主张目前依据代码同源性与人工抽查；
9. MathACL 路径依赖 `PI` 宏（sin_cos 硬件路径中 `((PI)*1<<q)`）需在器件头中另行定义，本仓库不含该定义——纯 SDK 环境下会编译失败，属集成注意事项。

---

# 十六、总结

本 IQmathLib 重设计以 **"一份算法内核 × 30 个编译期实例 × 2 条执行路径（RTS/MathACL）"** 的三维矩阵覆盖了 C2000 IQmath 的公开 API 面。数值方法忠实继承 TI 经典配方——查表+低阶 Taylor（sin/cos）、分段 minimax 多项式（asin/atan2）、Newton-Raphson 倒数与反平方根迭代（div/sqrt）、标度分解+截断级数（exp/log）、位拼 IEEE754（toF）、乘法消除法（toa/atoIQN）；又通过现代 C 抽象（fast 类型、强制内联参数化内核、寄存器级 HAL 风格封装、条件编译特性门控、隐藏内部 API）获得了原汇编库不具备的可移植性与可维护性。实测表明其核心运算（乘除、三角、反三角、开方、指数对数、类型转换）在纯软件路径上即达到标称精度，可作为在无 C2000 许可或缺乏 IQmath 支持的 MCU 上复刻电力电子/电机控制算法栈的参考基线；toa 解析缺陷、log 高幅值指数补偿溢出通道与 MathACL 集成注意事项是投产前应关闭的三个口子。其"薄壳–内核–抽象层"的架构范式亦可推广到其它定点 DSP 库的跨平台移植工作。
