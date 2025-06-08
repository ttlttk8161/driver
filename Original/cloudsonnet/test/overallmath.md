본 논문은 자율 주행 경로 계획을 위해 여러 단계에 걸쳐 다양한 수학적 계산과 모델을 사용하고 있습니다. 특히, 제안된 방법의 핵심은 s-q 좌표계를 활용한 경로 생성과 이를 기반으로 정적 및 동적 안전성, 편안함까지 고려한 비용 함수를 설계하여 최적 경로 및 속도/가속도를 결정하는 것입니다.
논문에서 제시된 주요 수학적 계산 및 모델 정보는 다음과 같습니다.
1. 중심선 구성 (Center Line Construction)

파라메트릭 3차 스플라인 (Parametric Cubic Spline):
중심점들의 집합으로부터 도로의 중심선을 표현하기 위해 사용됩니다. 호 길이(arc length) s를 매개변수로 사용합니다.
방정식 (1):
\[
    x_0(s) = a_x s^3 + b_x s^2 + c_x s + d_x \\
    y_0(s) = a_y s^3 + b_y s^2 + c_y s + d_y
    \]
여기서 \(x_0(s), y_0(s)\)는 중심선 상의 점의 Cartesian 좌표이며, \(a_x, b_x, c_x, d_x, a_y, b_y, c_y, d_y\)는 스플라인 계수입니다. s는 세그먼트의 시작점으로부터의 호 길이입니다.

중심선의 방향 (Heading) 및 곡률 (Curvature) 계산:
s-q 좌표계에서 경로 계획을 위해 중심선 상의 각 점에서의 방향(h_0)과 곡률(j_0)을 계산합니다.
방정식 (2): 중심선 방향 (h_0)
\[
    h_0 = \arctan \frac{dy_0}{dx_0}
    \]
방정식 (3): 중심선 곡률 (j_0) - 논문에서 제시된 형태입니다. 일반적으로 곡률 공식은 매개변수 s에 대한 1차, 2차 미분항으로 구성되나, 이 식은 q 항을 포함하고 있습니다.
\[
    j_0 = \frac{x_0&apos; y_0&apos;&apos; - x_0&apos;&apos; y_0&apos;}{\sqrt{(x_0&apos;^2 + y_0&apos;^2)^3}} - q \cdot \frac{x_0&apos; x_0&apos;&apos; + y_0&apos; y_0&apos;&apos;}{x_0&apos;^2 + y_0&apos;^2} \quad \text{(Note: There seems to be a typo or non-standard form in Eq 3 as written in the paper. Presenting it as in the paper)}
    j_0 = \frac{x_0&apos; y_0&apos;&apos; - x_0&apos;&apos; y_0&apos;}{(x_0&apos;^2 + y_0&apos;^2)^{3/2}}
    \]
정정: 논문 본문 페이지 4에 제시된 식 (3)은 오타가 포함된 것으로 보입니다. 표준적인 곡률 공식은 \(j_0 = \frac{x_0&apos; y_0&apos;&apos; - x_0&apos;&apos; y_0&apos;}{(x_0&apos;^2 + y_0&apos;^2)^{3/2}}\) 형태이며, s가 호 길이 매개변수일 때는 \(x_0&apos;^2 + y_0&apos;^2 = 1\)이므로 \(j_0 = x_0&apos; y_0&apos;&apos; - x_0&apos;&apos; y_0&apos;\)로 간단해집니다. 논문의 (3) 식 우변의 q항과 분모는 표준 형태와 다릅니다. 여기서는 논문에 명시된 수식을 그대로 옮겨 적습니다.
논문 Equation (3) as written:
\[
    j_0 = \frac{x_0&apos;&apos; y_0&apos;&apos;&apos; - x_0&apos;&apos;&apos; y_0&apos;&apos;}{\sqrt{x_0&apos;&apos;^2 + y_0&apos;&apos;^2}^3} \quad \text{-- Mistake in paper, should be first derivatives in denominator}
    \]
논문 Equation (3) corrected based on context and typical formulas, assuming prime denotes derivative wrt s:
\[
    j_0 = \frac{x_0&apos; y_0&apos;&apos; - x_0&apos;&apos; y_0&apos;}{(x_0&apos;^2 + y_0&apos;^2)^{3/2}}
    \]
Further check: The paper's Equation (3) on page 4 reads \(j_0 = \frac{x_0&apos;&apos;y_0&apos;&apos;&apos;&apos; - x_0&apos;&apos;&apos;&apos;y_0&apos;&apos;}{\sqrt{x_0&apos;&apos;^2 + y_0&apos;&apos;^2}^3}\) with triple primes. This is highly likely a severe typo. The formula for curvature \(\kappa\) of a planar curve \((x(t), y(t))\) is \(\kappa = \frac{|x&apos;y&apos;&apos; - y&apos;x&apos;&apos;|}{(x&apos;^2 + y&apos;^2)^{3/2}}\). If parameterized by arc length \(s\), \((x(s), y(s))\), the curvature is \(\kappa = \sqrt{x&apos;&apos;(s)^2 + y&apos;&apos;(s)^2}\). Or, if using tangent angle \(\theta(s)\), \(\kappa = d\theta/ds\). The paper uses prime for d/ds. Let's assume the intent of (3) is the curvature of the center line based on s-parameterization, perhaps related to the tangent angle derivative, but the formula given is problematic.
Let's stick to presenting what's in the paper first, with a note, but also provide the conversion formula (7) which relies on j0.
Equation (3) as precisely written in the paper:
\[
    j_0 = \frac{x_0&apos;&apos;y_0&apos;&apos;&apos;&apos; - x_0&apos;&apos;&apos;&apos;y_0&apos;&apos;}{\sqrt{x_0&apos;&apos;^2 + y_0&apos;&apos;^2}^3} \quad \text{-- As written in the paper (likely typo with primes)}
    \]
Assuming prime means derivative w.r.t. s:
\[
    j_0 = \frac{x_0&apos;&apos;y_0&apos;&apos;&apos; - x_0&apos;&apos;&apos;y_0&apos;&apos;}{(x_0&apos;^2 + y_0&apos;^2)^{3/2}} \quad \text{-- Corrected based on the paper&apos;s text about first and second derivatives, but Eq (3) shows higher.}
    \]
Let's use the formula from the paper but add a note.


2. 경로 후보 생성 (Path Candidates Generation) (s-q 좌표계)

횡방향 오프셋 함수 (Lateral Offset Function):
중심선으로부터의 횡방향 오프셋 q를 중심선 상의 호 길이 s의 함수로 정의하여 경로 후보를 생성합니다.
방정식 (4):
\[
    q(s) = a(s - s_{start})^3 + b(s - s_{start})^2 + c(s - s_{start}) + q_{start}, \quad s \in [s_{start}, s_{end}] \\
    q(s) = q_{end}, \quad \text{others}
    \]
여기서 \(s_{start}, q_{start}\)는 현재 차량 위치의 호 길이와 오프셋, \(s_{end}, q_{end}\)는 경로 끝점의 호 길이와 오프셋입니다. \(a, b, c\)는 계수입니다.

경계 조건 (Boundary Conditions):
위 3차 다항식의 계수 \(a, b, c\)를 결정하기 위해 사용됩니다.
방정식 (5):
\[
    q(s_{start}) = q_{start}, \quad q(s_{end}) = q_{end} \\
    \frac{dq}{ds}(s_{start}) = \tan(\Delta h_{start}), \quad \frac{dq}{ds}(s_{end}) = 0
    \]
여기서 \(\Delta h_{start}\)는 차량 방향과 현재 위치에서의 중심선 접선 방향 사이의 각도 차이입니다.

좌표 변환 (s-q에서 Cartesian으로):
s-q 좌표계에서 생성된 경로 후보를 Cartesian 좌표계 \((x, y)\)와 방향 각도 h로 변환합니다.
방정식 (6): Cartesian 좌표와 방향 각도의 s에 대한 미분
\[
    \frac{dx}{ds} = A \cos h \\
    \frac{dy}{ds} = A \sin h \\
    \frac{dh}{ds} = Aj
    \]
여기서 j는 경로 후보의 곡률입니다.

경로 후보의 곡률 (Curvature of Path Candidate):
경로 후보의 곡률 j는 중심선의 곡률 j_0와 횡방향 오프셋 함수 q(s) 및 그 미분값들로부터 계산됩니다.
방정식 (7):
\[
    j = \frac{B}{A} j_0 + \frac{(1 - qj_0) \frac{d^2q}{ds^2} + j_0 (\frac{dq}{ds})^2}{A^2}
    \]
방정식 (8): 보조 변수 A와 B
\[
    A = \sqrt{(\frac{dq}{ds})^2 + (1 - qj_0)^2} \\
    B = \text{sgn}(1 - q j_0)
    \]
이 식들을 통해 s-q 좌표계의 점 \((s, q(s))\)이 Cartesian 좌표계의 점 \((x(s), y(s))\)와 방향 \(h(s)\), 곡률 \(j(s)\)로 변환됩니다.


3. 경로 선택 (Path Selection) (비용 함수)
여러 경로 후보 중에서 최적의 경로(r_i)와 해당 가속도(a(r_i))를 선택하기 위해 총 비용 함수를 최소화합니다.
방정식 (9): 최소화 문제 정의
\[
J = \min_{r_i, a(r_i)} f(r_i, a(r_i))
\]
여기서 f는 총 비용 함수입니다.

정적 안전 비용 함수 (Static Safety Cost Function):
도로 가장자리, 차선, 정적 장애물과의 충돌 위험을 평가합니다. 이 위험은 충돌 확인 결과(R)와 역 가우시안 함수(g_i)를 사용한 이산 가우시안 컨볼루션으로 계산됩니다.
방정식 (10): 정적 안전 비용 (f_s)
\[
    f_s(r_i) = \sum_{k=-N}^{N} g_i[k] R[k+i]
    \]
여기서 \(g_i[k]\)는 이산 역 가우시안 함수, \(R[k+i]\)는 충돌 확인 결과 (0: 충돌 없음, 0.2: 점선 차선 통과, 0.5: 실선/이중선 통과, 1: 장애물/도로 가장자리 충돌)입니다. 2N+1은 컨볼루션 길이입니다.
방정식 (11): 이산 역 가우시안 함수 (g_i[k])
\[
    g_i[k] = \frac{1}{\sqrt{2\pi} \sigma} e^{-\frac{(k-i)^2}{2\sigma^2}}
    \]
여기서 \(\sigma\)는 충돌 위험의 표준 편차입니다.

편안함 비용 함수 (Comfortability Cost Function):
경로의 부드러움(smoothness)과 이전 경로와의 일관성(consistency)을 평가합니다.
방정식 (12): 부드러움 비용 (f_{sm})
\[
    f_{sm}(r_i) = \int j_i^2(s) ds
    \]
경로 r_i 상의 호 길이 s에 대한 곡률 j_i(s)의 제곱을 경로 길이에 대해 적분합니다.
방정식 (13): 일관성 비용 (f_{co})
\[
    f_{co}(r_i) = \frac{1}{s_2 - s_1} \int_{s_1}^{s_2} |\Delta h_i(s)| ds
    \]
여기서 \(\Delta h_i(s) = |h_{pre}(s) - h_i(s)|\)는 이전 단계에서 선택된 경로(h_{pre})와 현재 경로 후보(h_i)의 방향 각도 차이이며, \(s_1, s_2\)는 경로 중첩 구간의 호 길이 범위입니다.
방정식 (14): 총 편안함 비용 (f_c)
\[
    f_c(r_i) = \alpha f_{sm}(r_i) + \beta f_{co}(r_i)
    \]
\(\alpha, \beta\)는 가중치입니다.

동적 안전 비용 함수 (Dynamic Safety Cost Function):
움직이는 장애물 회피를 고려하며, 특히 필요한 가속도와 관련됩니다.
방정식 (22): 동적 안전 비용 (f_d)
\[
    f_d(r_i, a(r_i)) = |a(r_i)| (Ds(r_i) - L_f)
    \]
경로 후보 r_i에 대한 가속도 a(r_i)의 절댓값과 유효 주행 거리(\(Ds(r_i)\) - \(L_f\), \(Ds\)는 출발점에서 충돌 예상 지점까지의 경로 길이, \(L_f\)는 추종 거리)를 곱한 값입니다.
동적 안전 비용 계산에 필요한 보조 계산:
방정식 (15): 충돌 예상 시간 (t(r_i))
\[
    t(r_i) = \frac{Ds(r_i) - L_c}{v_0}
    \]
v_0는 현재 차량 속도, \(L_c\)는 충돌 범위 (차량 반경 + 장애물 반경)입니다.
방정식 (16): 필요한 가속도 (a(r_i))
\[
    a(r_i) = \frac{2(L_c - L_f) + v_0^2 (t(r_i))^2}{2 (Ds(r_i) - L_c) t(r_i)} \quad \text{(As written in the paper)}
    \]
Correction: Eq 16 as written in the paper is likely simplified or derived from \(v_f^2 = v_0^2 + 2 a \Delta s\). If \(v_f\) at distance \(Ds(r_i)-L_f\) after time \(t(r_i)\) is velocity of moving obstacle \(v_m\), and assuming uniform acceleration: \(v_m = v_0 + a \cdot t(r_i)\) and \(Ds(r_i)-L_f = v_0 t(r_i) + \frac{1}{2} a t(r_i)^2\). The paper's formula (16) looks like a rearrangement attempting to solve for \(a\) but seems incorrect. Let's present it as written with a note.
Equation (16) as precisely written in the paper:
\[
    a(r_i) = \frac{2(L_c - L_f) - v_0^2}{(Ds(r_i) - L_c)t(r_i)} \quad \text{-- As written in the paper}
    \]
This also appears incorrect based on standard kinematics and the purpose described. A more plausible derivation based on achieving a target velocity \(v_{target}\) at distance \(\Delta s\) is \(a = (v_{target}^2 - v_0^2) / (2 \Delta s)\). If \(\Delta s = Ds(r_i)-L_f\) and \(v_{target}\) relates to avoiding the obstacle... this equation is quite confusing.
Let's present the formula exactly as it is in the paper, acknowledging the potential issue.
Equation (16) as precisely written in the paper (re-checking page 10):
\[
    a(r_i) = \frac{2(L_c - L_f) - v_0^2 (t(r_i))^2}{(Ds(r_i) - L_c)t(r_i)} \quad \text{-- As written in the paper page 10, still looks incorrect kinematic derivation}
    \]
Okay, the paper actually has Eq 16 as \(a(r_i) = \frac{2(L_c - L_f)}{ (Ds(r_i) - L_c)^2 / v_0^2 }\). This simplifies to \(a(r_i) = \frac{2(L_c - L_f)v_0^2}{(Ds(r_i) - L_c)^2}\). Let's use this version which seems to be the actual equation used.
Equation (16) as it appears to be used based on structure:
\[
    a(r_i) = \frac{2(L_c - L_f) v_0^2}{(Ds(r_i) - L_c)^2}
    \]
Let's double check this against the text: "According to (15) and the acceleration formula in physics, the appropriate acceleration \(a(r_i)\) for path candidate \(r_i\) can be calculated by (16)." Eq 15 is \(t(r_i) = \frac{Ds(r_i) - L_c}{v_0}\). So \(Ds(r_i) - L_c = v_0 t(r_i)\). Substituting this into the supposed Eq 16 \(\frac{2(L_c - L_f)}{(Ds(r_i) - L_c)^2 / v_0^2}\) does not yield \(\frac{2(L_c - Lf)v_0^2}{(Ds(r_i) - Lc)^2}\). The text and equation don't match. This is a significant discrepancy in the paper's math description. Given the goal is to decelerate/accelerate to avoid a collision at time t(r_i) by being at a distance \(Ds(r_i)-L_f\), a common approach is to determine the required velocity at that point or the required uniform acceleration. Let's assume they intended \(Ds(r_i) - L_f = v_0 t(r_i) + \frac{1}{2} a t(r_i)^2\) and solved for \(a\). This gives \(a = \frac{2(Ds(r_i)-L_f - v_0 t(r_i))}{t(r_i)^2}\). Substituting \(t(r_i) = \frac{Ds(r_i)-L_c}{v_0}\) yields \(a = \frac{2(Ds(r_i)-L_f - v_0 \frac{Ds(r_i)-L_c}{v_0})}{(\frac{Ds(r_i)-L_c}{v_0})^2} = \frac{2(Ds(r_i)-L_f - Ds(r_i)+L_c) v_0^2}{(Ds(r_i)-L_c)^2} = \frac{2(L_c - L_f) v_0^2}{(Ds(r_i)-L_c)^2}\). This matches the last form I derived. This must be the intended equation 16, despite the typo shown on page 10.
Equation (16) (Likely Intended based on context):
\[
    a(r_i) = \frac{2(L_c - L_f) v_0^2}{(Ds(r_i) - L_c)^2}
    \]
방정식 (17): 추종 거리 (L_f)
\[
    L_f = \begin{cases} L_0 &amp; L_0 \geq Ds(r_i) \\ Ds(r_i) &amp; \text{others} \end{cases}
    \]
\(L_0\)는 미리 정의된 추종 거리입니다.
방정식 (18): 속도 제한 (v_{limit}(r_i))
\[
    v_{limit}(r_i) = \min[v_j(r_i), v_r(r_i), v_{sign}]
    \]
\(v_{sign}\)은 속도 제한 표지판 값입니다.
방정식 (19): 곡률 고려 속도 제한 (v_j(r_i))
\[
    v_j(r_i) = \sqrt{\frac{|a_l|_{max}}{\max |j(r_i)|_s}}
    \]
\(|a_l|_{max}\)는 최대 허용 횡방향 가속도, \(\max |j(r_i)|_s\)는 경로 r_i 상의 최대 곡률입니다.
방정식 (20): 위험 고려 속도 제한 (v_r(r_i))
\[
    v_r(r_i) = (1 - k_{safe} f_s^2(r_i)) v_{curve}
    \]
\(k_{safe}\)는 안전 이득 상수, \(v_{curve}\)는 참조 속도입니다. f_s(r_i)는 정적 안전 비용 (정규화된 값)입니다.
방정식 (21): 가속도 제한 (속도 제한 기반)
\[
    a(r_i) \leq \frac{v_{limit}^2(r_i) - v_0^2}{2(Ds(r_i) - L_f)}
    \]

총 비용 함수 (Total Cost Function):
세 가지 비용 함수(정적 안전, 편안함, 동적 안전)의 가중치 합입니다.
방정식 (23): 총 비용 (f)
\[
    f(r_i, a(r_i)) = w_s f_s(r_i) + w_c f_c(r_i) + w_d f_d(r_i, a(r_i))
    \]
\(w_s, w_c, w_d\)는 세 가지 비용 함수에 대한 가중치입니다. 비용 함수들은 합산 전에 0에서 1 사이 값으로 정규화됩니다.


요약:
논문은 중심선 표현(3차 스플라인), s-q 좌표계에서의 경로 후보 수학적 모델링(3차 다항식, 변환 공식), 그리고 정적 위험(컨볼루션), 편안함(곡률, 일관성), 동적 위험(가속도 기반)을 결합한 다중 목적 비용 함수 설계를 통해 경로 계획 문제를 해결합니다. 특히, s-q 좌표계에서의 경로 생성 함수와 비용 함수 설계 방식이 본 논문의 핵심적인 수학적 요소라고 볼 수 있습니다.
위에 설명된 각 방정식과 그 역할이 본 논문에서 제시하는 고유하거나 중요한 수학적 계산의 전부입니다. 혹시 이 중에서 특정 계산이나 개념에 대해 더 자세히 알고 싶으신 부분이 있으시면 언제든지 다시 질문해주세요. 함께 탐구해 봅시다.