# Hash-based Bag-of-Words (loop-closure retrieval)

이 문서는 `src/mapper/hash_bow.hpp`의 이론적 배경을 정리한다. Galvez-Lopez의
DBoW2와 Usenko et al.의 basalt를 참고해 재구현한 것으로, 키프레임 사이의
loop-closure 후보를 빠르게 검색하는 것이 목적이다.

**관련 코드**

- `src/mapper/hash_bow.hpp` — `HashBow<N>`
- `src/mapper/descriptor_extractor.hpp` — 256-bit ORB descriptor 추출
- `src/mapper/mapper.cpp` — 키프레임 등록 (`compute_bow` → `add_to_database`)

**표기**: descriptor는 $N$비트 이진 벡터 $d \in \{0,1\}^N$ (여기서 $N = 256$).
word는 `uint32_t` 정수, BoW 벡터는 (word, weight) 쌍의 희소 벡터로 표현한다.

---

## 1. 왜 Bag-of-Words인가

Loop closure는 "지금 보고 있는 장면을 과거에 이미 방문한 적이 있는가"를 묻는
문제다. 매 키프레임마다 과거의 모든 키프레임과 이미지를 직접 매칭하면
$O(n^2)$이 되어 지도가 커질수록 감당할 수 없다.

Bag-of-Words는 이미지 하나를 "어떤 시각 단어(visual word)들이 몇 번
등장했는가"라는 히스토그램으로 요약한다. 단어의 **순서와 위치를 버리고** 등장
빈도만 남기기 때문에("bag") 이미지가 고정 길이의 희소 벡터가 되고, 두 이미지의
유사도는 벡터 내적 한 번으로 계산된다. 텍스트 검색의 역색인(inverted index)을
그대로 빌려와, 질의 이미지와 단어를 공유하는 키프레임만 후보로 추려낼 수 있다.

---

## 2. 전통적 vocabulary와의 차이

전통적 DBoW는 대량의 학습용 descriptor를 k-means로 계층 군집화해 **vocabulary
tree**(사전)를 미리 만든다. 각 descriptor는 트리를 타고 내려가 가장 가까운 leaf,
즉 visual word로 양자화된다. 성능은 좋지만 다음의 부담이 있다:

- 오프라인 학습 단계와 수십 MB짜리 사전 파일이 필요하다.
- 사전이 학습 도메인에 묶인다 (실내에서 학습한 사전이 실외에서 약해짐).
- descriptor마다 트리 탐색을 해야 한다.

basalt가 택한(그리고 여기서 따르는) 대안은 **학습 없는 해싱**이다. 사전을 만드는
대신, descriptor의 비트 일부를 그대로 정수 word로 쓴다. 학습·사전 파일·도메인
의존성이 모두 사라지고, word 계산이 비트 몇 개 읽는 것으로 끝난다. 대가로 word가
descriptor 공간을 균등 분할하지 못하지만(학습된 사전보다 변별력이 낮다),
loop-closure **후보 검색**에는 충분하다 — 최종 판정은 뒤따르는 기하 검증이 한다.

---

## 3. Word: descriptor 비트에서 정수로

핵심 발상은 단순하다. $N$비트 descriptor에서 미리 정한 $b$개의 비트를 골라
그대로 이어 붙여 $b$비트 정수를 만들면, 그게 곧 word다. 즉 word는 descriptor를
$b$차원 해밍 부분공간으로 사영한 값이다. 서로 가까운(해밍 거리가 작은)
descriptor는 대체로 같은 비트값을 가지므로 같은 word로 떨어질 확률이 높다 —
학습된 vocabulary가 하던 양자화를 근사한다.

### 3.1 어떤 비트를 고를 것인가 — 순열

인접한 비트를 그냥 앞에서부터 $b$개 쓰지 않고, descriptor 전체에 걸쳐
**결정론적으로 섞은 순열**에서 앞 $b$개를 뽑는다. ORB 같은 descriptor는 이웃
비트끼리 상관이 있을 수 있어, 연속 비트만 쓰면 정보가 편중된다. 순열로 전
영역에서 비트를 흩뿌리면 word가 descriptor를 더 고르게 대표한다.

`detail::make_bit_permutation<N>`은 고정 seed의 LCG(선형 합동 생성기)로
Fisher-Yates 셔플을 돌려 `constexpr` 순열 배열을 컴파일 타임에 만든다:

$$\text{state} \leftarrow \text{state}\cdot a + c, \qquad
  j \leftarrow \left\lfloor \tfrac{\text{state}}{2^{33}} \right\rfloor \bmod (i+1)$$

seed가 고정(`kPermutationSeed`)이라 매 실행·매 키프레임에서 **같은 순열**이
나온다는 점이 중요하다. 그래야 서로 다른 키프레임의 word가 같은 규칙으로
계산되어 비교 가능하다.

### 3.2 `compute_hash`

```
hash = 0
for i in [0, num_word_bits):
    if descriptor[perm[i]]:
        hash |= (1 << i)
```

순열의 앞 `num_word_bits_`개 위치를 읽어 정수로 채운다. word 비트 수는
`min(요청값, kMaxWordBits=32, N)`으로 제한된다 — word가 `uint32_t`이기 때문.

**비트 수 $b$의 트레이드오프**: word 공간의 크기는 $2^b$다.

- $b$가 크면 word가 세분화돼 서로 다른 장면을 잘 구별하지만(변별력↑), 같은
  장면도 조명·시점 변화로 다른 word에 떨어지기 쉬워진다(재현율↓).
- $b$가 작으면 그 반대다 — 관대하게 매칭되지만 변별력이 떨어진다.

이 코드베이스는 256비트 descriptor에 $b = 24$ (`kHashWordBits`,
`mapper.cpp`)를 쓴다 → word 공간 $2^{24} \approx 1.6\times10^7$.

---

## 4. BoW 벡터: 빈도와 정규화

`compute_bow`는 한 키프레임의 descriptor 집합을 BoW 벡터로 바꾼다.

1. **Term frequency (TF)**: 각 word가 몇 번 등장했는지 센다. word를 공유하는
   descriptor가 많을수록 그 word의 가중치가 커진다.

   $$\text{tf}(w) = \#\{\, d : \text{hash}(d) = w \,\}$$

2. **L1 정규화**: 가중치 벡터를 $\ell_1$ 노름으로 나눈다.

   $$a_w = \frac{\text{tf}(w)}{\sum_{w'} \text{tf}(w')}, \qquad \sum_w a_w = 1$$

   정규화가 없으면 특징점이 많은 키프레임이 무조건 높은 점수를 받는다.
   $\ell_1$ 정규화는 descriptor 개수의 절대량을 지우고 **분포**만 비교하게 한다.

> 참고: 전통적 DBoW는 여기에 IDF(inverse document frequency) 가중치를 곱해 흔한
> word를 깎는다. 이 구현은 학습된 사전이 없어 IDF를 두지 않고 TF만 쓴다.

---

## 5. Inverted index와 질의

### 5.1 데이터베이스

`add_to_database`는 BoW 벡터를 **역색인**에 흩어 넣는다: word → 그 word를 가진
(keyframe_id, weight) 목록.

$$\text{index}[w] = [(k_1, a^{(1)}_w),\ (k_2, a^{(2)}_w),\ \dots]$$

질의 때 질의 벡터의 word들만 훑으면, 단어를 하나라도 공유하는 키프레임에만
접근하게 된다 — 전체 키프레임을 순회하지 않는다.

### 5.2 L1 유사도 점수

`query`는 DBoW의 $\ell_1$ 점수를 쓴다. 정규화된 두 벡터 $a, b$
($\lVert a\rVert_1 = \lVert b\rVert_1 = 1$)에 대해

$$s(a, b) = 1 - \tfrac{1}{2}\lVert a - b \rVert_1 \;\in\; [0, 1]$$

$s=1$이면 완전 동일, $s=0$이면 공유 word가 없음. 이 점수는 공유 word만으로
계산할 수 있다는 게 구현의 핵심이다. 각 word $i$의 기여

$$|a_i - b_i| - |a_i| - |b_i|$$

를 보면, **한쪽에만 있는 word**($b_i = 0$)는
$|a_i| - |a_i| - 0 = 0$으로 사라진다. 따라서

$$\sum_{\text{all } i}\big(|a_i - b_i| - |a_i| - |b_i|\big)
  = \lVert a - b\rVert_1 - \lVert a\rVert_1 - \lVert b\rVert_1
  = \lVert a - b\rVert_1 - 2,$$

이고 이 합은 **공유 word만 순회해도** 얻어진다. 정리하면

$$s(a,b) = -\tfrac{1}{2}\sum_{\text{shared } i}\big(|a_i - b_i| - |a_i| - |b_i|\big).$$

코드가 그대로 이 식이다: 역색인에서 만난 공유 word마다

```
scores[id] += |weight - entry.weight| - |weight| - |entry.weight|
```

를 누적하고, 마지막에 `-score / 2`로 $[0,1]$ 점수를 낸다.

### 5.3 시간 제약과 상위 K 선택

- `newest_allowed_id`: 이 id보다 새로운 키프레임은 후보에서 제외한다. 방금 만든
  키프레임이 바로 직전 키프레임들과 매칭돼 자기 자신에 가까운 것을 loop로 오인하는
  것을 막는다 (loop closure는 시간적으로 충분히 떨어진 과거를 찾아야 한다).
- `std::partial_sort`로 점수 상위 `num_results`개만 추린다. 이들이 이후 기하
  검증(feature 매칭 + RANSAC 등)으로 넘어갈 loop 후보다.

---

## 6. 전체 흐름 요약

```
키프레임 이미지
   │  detect_and_compute
   ▼
[256-bit ORB descriptors]
   │  compute_bow: hash(순열 24비트) → TF → L1 정규화
   ▼
BoW 벡터  [(word, weight), ...]
   ├── add_to_database ──▶ inverted index (word → [(kf, weight)])
   └── query ──▶ 공유 word L1 점수 ──▶ 상위 K loop 후보 ──▶ (기하 검증)
```

이 모듈은 loop **후보 검색**까지만 담당한다. 후보를 실제 loop로 확정하고
pose-graph에 제약을 넣는 검증·최적화는 `Mapper::process`의 TODO로 남아 있다.
