# 개발 워크플로 & 반복 실수

## 개발 워크플로

```
1. 요청 파악
   └─ Grep으로 관련 파일/함수 위치 확인 → 필요한 부분만 Read

2. 변경 범위 결정
   └─ 최소 변경 원칙 — 영향 범위를 최소화

3. 구현
   ├─ 새 파일       → Write
   └─ 기존 파일     → Read 로 정확한 내용 확인 → Edit

4. 패턴 준수 체크
   ├─ ui_queue.post() 사용 여부 (백그라운드 스레드)
   ├─ Python 3.8 타입 힌트 호환 (from __future__ import annotations)
   └─ DearPyGUI 유니코드/탭바 규칙

5. README.md & CLAUDE.md 업데이트
   └─ 새 파일, 새 기능, 변경 파라미터 반영
```

---

## 세션 관리 (토큰 절약)

| 상황 | 액션 |
|------|------|
| 한 기능 구현 완료 후 | `/compact` 로 컨텍스트 압축 |
| 완전히 새로운 작업 시작 | `/clear` 후 새 세션 |
| 큰 파일 읽기 필요 | `Grep` 으로 위치 먼저 → `offset+limit` 으로 부분 Read |

---

## 반복 실수 목록

| 상황 | 잘못된 접근 | 올바른 접근 |
|------|------------|------------|
| 백그라운드 UI 업데이트 | `dpg.set_value()` 직접 | `ui_queue.post()` |
| Python 3.8 타입 힌트 | `list[str]`, `dict[str,int]` | `from __future__ import annotations` |
| DearPyGUI 탭바 | `add_tab_bar` 사용 | 버튼+show/hide |
| 버튼 라벨 | `↺`, 이모지 등 유니코드 | ASCII 텍스트 |
| 파일 편집 | 기억 의존해 old_string 작성 | 편집 전 Read 확인 |
| 패널 → app.py 접근 | `import app` | `init(callback)` 주입 |
| DearPyGUI 아이템 접근 | 태그 직접 사용 | `does_item_exist()` 체크 |
| 큰 파일 전체 읽기 | `Read(file)` 한 번에 | `Grep` → `Read(offset, limit)` |
| 로그 빈도 | 매 tick `log.append()` | `status_cb`로 `set_value` 직접 업데이트 |
| 동적 DPG 위젯 재생성 | 부모 없이 아이템 추가 | `with dpg.group(parent=...)` 컨텍스트 안에서 추가 |
| mvInputText 스크롤 | `set_y_scroll` / `get_y_scroll_max` | 미지원 — ChildWindow에만 사용 가능 |

---

## RTF 최적화 — 병목 분석 이력

RTF(Real-Time Factor) = sim_time / wall_clock_time. 1.0 = 실시간 동기화.

### 개선 전후 수치

| 단계 | RTF (Fixed Step) | 변경 내용 |
|------|-----------------|-----------|
| 초기 | ~0.2 | 기준선 |
| 클라이언트 로그 제거 | ~0.3–0.4 | TCP 수신 로그, ManualControl 매 tick 로그 삭제 |
| 시뮬레이터 로그 제거 | ~0.4–0.5 | 시뮬레이터 측 로그 제거 |
| 이후 상한 | ~0.5 | 시뮬레이터 물리 연산이 남은 병목 |

### 클라이언트 측 최적화 내용

**① 로그 오버헤드 제거**
- `transport/tcp_thread.py`: 모든 TCP 패킷 수신 로그 → result_code != 0 일 때만 출력
- `transport/tcp_transport.py`: ManualControl / TransformControl 매 tick 로그 파라미터 제거
- Runner: 매 tick `log.append(f"pos=...")` → `status_cb(...)` 로 교체 (`set_value` 직접)

**② O(n) 경로 탐색 → 윈도우 탐색**
- `PathManager.get_local_path()`: 전체 경로 순회 → `_last_wp` ±5/+100 윈도우
- `PurePursuit.calculate_steering_angle()`: 전체 local_path 순회 → `_last_lfd_idx` 캐시

**③ log.append vs set_value 비용 차이**
- `log.append`: ui_queue 큐잉 + 텍스트 누적 + InputText DPG 재빌드
- `set_value`: ui_queue 큐잉 + 값 교체만 → 약 10배 이상 저렴

### 시뮬레이터 측 병목

클라이언트 최적화 후 남은 RTF 한계는 시뮬레이터 내부 물리 엔진 연산 시간이다.
클라이언트에서 더 최적화해도 RTF는 크게 개선되지 않는다.

---

## 파일 크기 가이드

| 줄 수 | 처리 방법 |
|-------|----------|
| ~500줄 | Read 전체 가능 |
| 500~1,000줄 | Grep으로 위치 → 부분 Read |
| 1,000줄 이상 | 클래스/책임 단위로 파일 분리 검토 |

현재 대형 파일: `app.py`, `panels/monitor.py`, `panels/lane_control_panel.py`
---

## TCP API Workflow

TCP API 紐낅꽭??`transport/message_schema.py`瑜??쏅씪?대컮 ?쇱썝(origin)?쇰줈 愿由ы븳??payload ?꾨뱶媛 諛붾뀌硫??꾩쓬 ?쒖꽌瑜??좊Ⅴ?덈떎.

1. `transport/message_schema.py` ?섏젙
2. `python tools/gen_tcp_docs.py`
3. `python tools/gen_tcp_docs.py --check`
4. `docs/tcp-api.md` diff 由щ럭

`docs/tcp-api.md`? generated file濡?媛꾩＜?섎?濡?吏곸젒 ?몄쭛?섏? ?딄퀬, ?꾪빆??schema ?섏젙 ???ъ깮?섏뼱?쇳븳??
## TCP API Checks

- `transport/message_schema.py`를 수정한 뒤 `python tools/gen_tcp_docs.py`로 [docs/tcp-api.md](/C:/Dev/MORAI-SimControl_v2.1/docs/tcp-api.md:1)를 재생성한다.
- 커밋 전 `python tools/gen_tcp_docs.py --check`를 실행해 schema, generated doc, protocol 정의가 서로 맞는지 확인한다.
- request payload 필드나 타입이 바뀌면 `python -m unittest tests.test_tcp_payloads`를 실행해 대표 패킷의 바이너리 결과가 유지되는지 확인한다.
