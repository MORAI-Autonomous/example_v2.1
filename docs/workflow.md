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

---

## 파일 크기 가이드

| 줄 수 | 처리 방법 |
|-------|----------|
| ~500줄 | Read 전체 가능 |
| 500~1,000줄 | Grep으로 위치 → 부분 Read |
| 1,000줄 이상 | 클래스/책임 단위로 파일 분리 검토 |

현재 대형 파일: `lane_controller.py` (분리 완료), `app.py`, `panels/monitor.py`
