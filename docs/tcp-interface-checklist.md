## TCP Interface Checklist

이 문서는 TCP 인터페이스를 새로 추가하거나 기존 인터페이스를 수정할 때의 시작점과 작업 순서를 정리한다.

### Source Of Truth

- TCP 명세의 시작점은 [transport/message_schema.py](/C:/Dev/MORAI-SimControl_v2.1/transport/message_schema.py:1)이다.
- request 필드, response 필드, 반복 필드, 설명은 먼저 여기서 수정한다.
- 문서와 helper 검증은 이 파일을 기준으로 생성된다.

### New Interface

1. [transport/message_schema.py](/C:/Dev/MORAI-SimControl_v2.1/transport/message_schema.py:1)에 request `MessageSpec`를 추가한다.
2. response가 있으면 같은 파일의 `RESPONSE_MESSAGES`에도 `MessageSpec`를 추가한다.
3. [transport/protocol_defs.py](/C:/Dev/MORAI-SimControl_v2.1/transport/protocol_defs.py:1)에 `MSG_TYPE_*` 상수와 필요한 format/size 상수를 추가한다.
4. [transport/tcp_transport.py](/C:/Dev/MORAI-SimControl_v2.1/transport/tcp_transport.py:1)에 send 함수와 payload builder를 추가한다.
5. response를 구조적으로 읽어야 하면 같은 파일에 parser를 추가한다.
6. response를 앱 레벨에서 처리해야 하면 [transport/tcp_thread.py](/C:/Dev/MORAI-SimControl_v2.1/transport/tcp_thread.py:1)에 분기를 추가한다.
7. UI나 runner에서 이 인터페이스를 실제 호출하는 경로를 연결한다.

### Existing Interface Change

1. 바뀐 필드를 [transport/message_schema.py](/C:/Dev/MORAI-SimControl_v2.1/transport/message_schema.py:1)에서 먼저 수정한다.
2. 변경된 필드 타입과 순서가 [transport/protocol_defs.py](/C:/Dev/MORAI-SimControl_v2.1/transport/protocol_defs.py:1)에 반영됐는지 확인한다.
3. [transport/tcp_transport.py](/C:/Dev/MORAI-SimControl_v2.1/transport/tcp_transport.py:1)의 send/parser가 새 schema와 맞는지 수정한다.
4. [transport/tcp_thread.py](/C:/Dev/MORAI-SimControl_v2.1/transport/tcp_thread.py:1)의 response 처리와 로그 포맷이 맞는지 확인한다.
5. 호출부 UI, runner, panel, CLI가 바뀐 필드를 모두 전달하는지 확인한다.

### Validation

- `python tools/gen_tcp_docs.py`
- `python tools/gen_tcp_docs.py --check`
- `python -m unittest tests.test_tcp_payloads`

### Rule Of Thumb

- 명세 변경은 항상 `message_schema.py`부터 시작한다.
- request payload 변경이면 `tcp_transport.py` builder와 golden payload test를 같이 본다.
- response payload 변경이면 `tcp_transport.py` parser와 `tcp_thread.py` 처리 분기를 같이 본다.
- 문서 수동 수정 대신 `gen_tcp_docs.py`로 [docs/tcp-api.md](/C:/Dev/MORAI-SimControl_v2.1/docs/tcp-api.md:1)를 다시 생성한다.
