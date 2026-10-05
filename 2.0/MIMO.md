# 8×8 MIMO damping of the plate

All numbers measured 2026-10-05 at bias $b = 0.30$ V.

## 1. The system

| | symbol | size | unit |
|---|---|---|---|
| sensor readings (input) | $y$ | 8 | counts, $0 \dots 1023$ |
| coil voltages (output) | $u$ | 8 | V, $0 \dots 2.5$ |
| mode coordinates | $q$ | 3 | counts |
| mode frequencies | $\omega_m = 2\pi f_m$ | 3 | rad/s |

The plate is rigid with three lightly damped modes ($Q > 433$, so undamped on control timescales):

$$\ddot q_m + \omega_m^2\, q_m = \omega_m^2 \sum_j A_{mj}\,(u_j - b) + \xi_m, \qquad y_i = \sum_m \Phi_{im}\, q_m + d_i + n_i$$

$\xi$ is the ambient forcing, $d_i$ a slowly drifting offset, $n_i$ sensor noise. Everything the controller needs is in three objects: $f$ (3 numbers), $\Phi$ ($8\times3$) and $A$ ($3\times8$).

## 2. The sign test gives $D$

Hold all coils at $b$. For each coil $j$ in turn, ramp it to $b+\Delta$, hold $T = 12$ s, ramp to $b-\Delta$, hold, return ($\Delta = 0.15$ V). The static response is

$$D_{ij} = \frac{\bar y_i^{(+)} - \bar y_i^{(-)}}{2\Delta} \quad [\text{counts/V}]$$

The ramp still rings the plate, and a ring never dies within a dwell. So $\bar y$ is not a plain mean; each dwell is fitted as

$$y_i(t) = \bar y_i + \sum_{m=1}^{3} \big(a_{im}\cos\omega_m t + c_{im}\sin\omega_m t\big)$$

and $\bar y_i$ is the fitted constant. Its uncertainty $\sigma_{ij}$ comes from the scatter of 1 s block means of the residual. The sign of every entry is physics: nothing is assumed.

$$D = \begin{pmatrix}
+81.7 & +32.8 & -85.1 & +7.9 & -8.5 & -3.0 & +1.8 & +4.3\\
-11.2 & -54.2 & +28.4 & -46.0 & +5.1 & -2.0 & +1.4 & +0.1\\
+52.8 & +28.1 & -84.6 & +6.2 & -6.0 & -0.9 & +1.5 & -14.3\\
-35.6 & -39.0 & +0.5 & -54.8 & +5.3 & -0.9 & +2.1 & -5.6\\
+0.6 & -2.8 & -0.6 & +0.5 & -33.9 & +5.2 & -0.6 & -1.0\\
-1.0 & -1.3 & +0.1 & +0.1 & +5.0 & +5.8 & -0.0 & -0.4\\
+6.4 & +6.2 & +0.3 & +0.2 & +0.6 & +0.1 & +0.3 & +12.0\\
+6.2 & +5.9 & +0.1 & +0.1 & +0.1 & +0.1 & +0.0 & +11.7
\end{pmatrix}$$

(rows = sensors a0–a7, columns = coils 0–7). Diagonal SNR $|D_{jj}|/\sigma_{jj}$: 87, 103, 75, 98, 4, 18, **1**, 37.

**Coil 6 does nothing.** $D_{66} = +0.3 \pm 0.5$, against $D_{67} = +12.0$ for its neighbour. A direct toggle test (0.10 ↔ 0.50 V, four cycles) gives a6 a shift of $0.00 \pm 0.00$ counts/V under coil 6 and $+10.9 \pm 0.6$ under coil 7.

## 3. The modes give $f$ and $\Phi$

From 300 s with nothing driven. $f_m$ is the spectral peak, agreed by the four corner sensors to $< 5\times10^{-5}$ Hz:

$$f = (0.74017,\ 0.99641,\ 1.65569)\ \text{Hz}$$

The corner shape of mode $m$ is the dominant eigenvector of the corners' cross-spectrum in a narrow band about $f_m$,

$$C^{(m)}_{ik} = \sum_{f \approx f_m} Y_i(f)\,Y_k^*(f), \qquad C^{(m)} \phi_m = \lambda_{\max}\,\phi_m ,$$

made real and scaled to $\lVert\phi_m\rVert = 2$. It is rank one to 0.999 or better. Any other sensor's entry is its regression on that mode, kept only if it is $4\sigma$ from zero over ten time segments.

$$\Phi_{\text{corners}} = \begin{pmatrix}
+1.031 & +1.235 & -0.187\\
-1.002 & +0.640 & +1.057\\
+1.151 & +1.175 & -1.214\\
-0.780 & +0.797 & -1.171
\end{pmatrix}, \qquad \Phi_{a4 \dots a7} \approx 0 \ (|\Phi| < 0.005)$$

Columns are, in order, a tilt (98 % pure), the normal translation $Z$ (94 %), and a second tilt that is only 66 % pure: a0 sits near its node. That is why $\Phi$ is measured rather than taken as $\pm1$ patterns. Sensors a4–a7 see essentially none of the three modes.

## 4. $A$ follows from $D$ and $\Phi$

A static coil voltage deflects each mode by $q_m = \sum_j A_{mj}(u_j - b)$, and the corners read $\Phi_c\, q$. So $D_c = \Phi_c A$, and

$$A = \Phi_c^{+}\, D_c$$

with $\Phi_c^+$ the pseudo-inverse of the $4\times3$ corner block. Because $A$ is built from the same $\Phi$, their signs cannot disagree: flipping a column of $\Phi$ flips the matching row of $A$.

$$A = \begin{pmatrix}
+45.8 & +41.7 & -41.1 & +36.9 & -6.4 & -0.0 & -0.5 & +1.0\\
+26.2 & -12.9 & -30.7 & -23.9 & -0.9 & -2.1 & +1.7 & -1.8\\
+20.0 & -1.1 & +4.5 & +6.2 & -1.2 & -0.9 & -0.2 & +5.4
\end{pmatrix}\ \text{counts/V}$$

Column significance $\lVert A_{\cdot j}\rVert / \sigma$: 75, 66, 70, 71, 4.5, 5.9, 3.8, 15. Singular values 85.4, 46.4, 18.8 (condition 4.5): the third mode is the hardest to push, and coil 0 does most of it.

A coil enters the law if its column is $3\sigma$ from zero. All eight pass, but coil 6 only just (3.8), on one entry that did not repeat between two passes. Treat it as zero until it is repaired.

## 5. Estimating $\dot q$: the Kalman filter

State $x = (q_1, \dot q_1, q_2, \dot q_2, q_3, \dot q_3, d_1 \dots d_8)$. Each mode advances by an exact rotation over one control step $\tau = 10$ ms,

$$\begin{pmatrix} q \\ \dot q \end{pmatrix}_{k+1} = \begin{pmatrix} \cos\omega\tau & \sin\omega\tau/\omega \\ -\omega\sin\omega\tau & \cos\omega\tau \end{pmatrix} \begin{pmatrix} q \\ \dot q \end{pmatrix}_k ,$$

and the measurement is $y = \Phi q + d + n$. Two noise levels, both measured from the same 300 s:

- $R_i$ = everything in sensor $i$'s spectrum that is not one of the three modes. $\sqrt{R}$ = 4.0, 1.5, 2.0, 1.2, 1.0, 0.7, 1.4, 0.8 counts.
- $Q_m = 2\omega_m^2\,\sigma_m^2 / T_a$, the white force that would grow mode $m$'s measured variance $\sigma_m^2$ in $T_a = 50$ s.

The fit is checked by $\chi^2/\text{dof} = e^\top S^{-1} e / n$ on the innovation $e$. On the passive record it averages **0.88** (1 is a perfect model).

## 6. The control law

$$\boxed{\,f = -K\,P\,\dot q, \qquad u - b = A_C^{+}\, f\,}$$

- $K = \mathrm{diag}(k_m)$, one gain per mode, ramped up from zero.
- $A_C$ is $A$ restricted to the usable coils, $A_C^+$ its minimum-norm inverse (balanced so no coil works harder than the others), and $P$ the projector onto the modes those coils can reach.

**Why it damps.** The power delivered to the modes is

$$W = \dot q^\top A_C\,(u-b) = -\dot q^\top P K P\,\dot q \le 0$$

for any $k_m > 0$, because $A_C A_C^+ = P$. This holds only if $A$ is right; a wrong sign in $A$ makes $W > 0$ and the loop pumps. That is why the sign test is the foundation.

**Limits.** Three constraints, all applied by scaling the whole vector $u-b$ by one factor so its direction, and therefore the sign of $W$, is preserved:

$$|u_j - b| \le 0.27\ \text{V}, \qquad 0 \le u_j, \qquad \sum_j u_j \le 3.6\ \text{V}$$

The last one is the supply. Measured today: once $\sum_j u_j \approx 4.1$ V the coils stop following their commands (three sweeps; a single coil is linear to 0.85 V when the others are low). At $b = 0.30$ V the sum at rest is 2.4 V.

**PID path.** A coil whose column of $A$ is unresolved is left out of $A_C$. It may instead get $u_j - b = -g_j\,\dot y_j$ from its own sensor, with $g_j$ carrying the sign of $D_{jj}$, since the power is $-D_{jj}\,g_j\,\dot y_j^2$. Per-channel feedback on all coils damps every mode only if

$$S = A\,\mathrm{diag}(g)\,\Phi, \qquad \tfrac12(S + S^\top) \succ 0 .$$

Today its eigenvalues are 0.16, 2.54, 4.75, so it is a valid fallback, though weak on the third mode.

**Budget ramp.** Each mode's gain is $w_m k_m$ with $\sum w = \text{const}$. The weights follow each mode's share of the dissipated power $k_m \dot q_m^2$, low-passed at 20 s. Shipped flat ($w_m = 1$).

## 7. Link and ADC

- Samples: each frame is the sum of $N = 16$ ADC scans, 216 frames/s. With several counts of noise as dither, that adds about $\tfrac12\log_2 N = 2$ bits. Measured noise above 5 Hz fell from 4–13 counts to 1.2–4.9.
- The ADC reference cannot be rescaled: readings span 475–1001 of 1023 and already touch the top, so the unused range is the bottom half. Using it needs an analog offset and gain.
- Commands: all eight voltages in one checksummed frame every step. Over 75 000 frames: 0 lost, 0 corrupted, 0 rejected.

## 8. Not established

- **No loop has been closed on this hardware state.** Everything above is open-loop measurement plus simulation (0.22× of open-loop motion on a plant built from the same $A$, which cannot reveal a wrong $A$).
- **The supply limit** is inferred from the sweeps and the owner's scope; nobody has read the supply's own indicator.
- **Coil 6** needs hands: check the voltage at its terminals (DAC channel 4).
- **$A$'s third row** rests almost entirely on coil 0; an error there matters most.
- The response of a4 to coil 4 ($-34$ counts/V) is slow and still creeping after 12 s. It is an overdamped direction, not a fourth mode.

---
*Records:* `data/20261005_132003_v2_dc.csv` (sign test), `data/20261005_141902_v2_census.csv` (modes, noise). *To redo everything:* `run.py measure`, then `run.py preflight`, then `run.py bench`.
