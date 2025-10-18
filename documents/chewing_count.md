# Eating Counting Method / 摂食カウント方法

MAR（Mouth Aspect Ratio）は以下の4点のランドマーク座標を使用して計算されます。  

| ランドマーク名 | MediaPipe Index | 説明 |
|----------------|------------------|------|
| 左口角 | 61 | 口の左端 |
| 右口角 | 291 | 口の右端 |
| 上唇中央 | 13 | 上唇の内側中央 |
| 下唇中央 | 14 | 下唇の内側中央 |

式：
```math
MAR = \frac{vertical distance (upper–lower lip)}{horizontal distance (left–right corner)}
```

MediaPipe Face Mesh の代表点で書くと：
```math
MAR = \frac{|P_{13} - P_{14}|}{|P_{61} - P_{291}|}
```

# Eating State Detection Method / 摂食状態検出方法
## Distance between landmarks
Each landmark coordinate is a 3D vector / 各ランドマーク座標は3次元ベクトル
```math
P = (x, y, z)
```

Distance between any two points / 任意の2点間の距離
```math
d(A,B) = \sqrt{(A_x - B_x)^2 + (A_y - B_y)^2 + (A_z - B_z)^2}
```

Distance from the tip of the nose to the right and left hands / 右手・左手それぞれの鼻先との距離
```math
d_{hR} = d(P_{hand,R}, P_{nose}) \\
d_{hL} = d(P_{hand,L}, P_{nose}) \\
Compare hR and hL and use the smaller one \\
d_h = min(d_{hR}, d_{hL})
```
$d_{hand,R}$ : Right hand landmark coordinate / 右手ランドマーク座標  
$d_{hand,L}$ : Left hand landmark coordinate / 左手ランドマーク座標  
$d_{nose}$ : Nose landmark coordinate / 鼻先ランドマーク座標

Nose-chin distance / 鼻先-顎先距離
```math
d_j = d(P_{nose}, P_{chin})
```
$P_{chin $ : Chin landmark coordinate / 顎先ランドマーク座標

Mouth opening amount / 口の開き具合
```math
Distance between upper and lower lips (vertical direction) / 上唇と下唇の距離（垂直方向）\\
d_m = d(P_{upper\_lip}, P_{lower\_lip}) \\

Distance between corners of mouth (horizontally) \\
d_h = d(P_{left\_corner}, P_{right\_corner}) \\

MAR (mouth aspect ratio) / 口の縦横比 \\
MAR = \frac{d_m}{d_h}
```

MAR Exponential Moving Average (EMA) / MAR指数移動平均
```math
MAR_{ema}(t) = \alpha \cdot MAR(t) + (1 - \alpha) \cdot MAR_{ema}(t-1) \\
Initial condition: mathrm{MAR}_{ema}(t_0) = \mathrm{MAR}(t_0)
```
$\alpha$ : ema_alpha parameter

## State detection conditions
Feeding state / 摂食
```math
Feeding \iff d_h < \theta_{feed}
```
$\theta_{feed}$ = feeding_distance_threshold

Speaking state / 発話
```math
Speaking \iff (MAR_{ema} > \theta_{speak}) \land \neg Chewing
```
$\theta_{speak}$ = speaking_mar_threshold

Chewing state / 咀嚼
```math
\text{Conditions for starting the chewing cycle (the moment the mouth opens)}
if \neg C_{open}(t-1) \land MAR_{ema}(t) > \theta_{high} \implies C_{open}(t) = True
```
$C_{open}(t)$ is a flag indicating that the cycle is in an open state. \\

```math
\text{End condition of the chewing cycle (the moment the mouth closes)}\\
if C_{open}(t-1) \land MAR_{ema}(t) < \theta_{low} \implies C_{open}(t) = False
```
```math
if\left\{
\begin{align*}
C_{open}(t -1 ) = True \\
C_{open}(t) = False \\
\delta t = t - t_{last} > T_{min}
\end{align*}
\right\}

\implies \text{ChewingCount} := \text{ChewingCount} + 1
```
| 記号                      | 意味                             |
| :---------------------- | :----------------------------- |
| $\mathrm{MAR}_{ema}(t)$ | MAR の指数移動平均値                   |
| $\theta_{high}$         | 咀嚼開口側閾値（`chewing_mar_high`）    |
| $\theta_{low}$          | 咀嚼閉口側閾値（`chewing_mar_low`）     |
| $C_{open}(t)$           | 現在の咀嚼サイクルが開状態かを表すブール値          |
| $t_{\text{last}}$       | 直近の咀嚼完了時刻                      |
| $T_{\min}$              | 最小咀嚼間隔（`min_chewing_interval`） |
| $\Delta t$              | 現在と前回の咀嚼完了時刻との差                |

Idle state / 静止
```math
Idle \iff \neg (Feeding \lor Speaking \lor Chewing)
```