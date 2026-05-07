# UDP Debug Scripts

예제 앱 본체와 분리한 UDP 분석/디버그 스크립트 보관 폴더입니다.

## Files

- `molit8_parser_rsa.py`
  - RSA 관련 기존 스크립트 4개를 하나로 통합한 standalone 도구
  - subcommand:
    - `parse`
    - `record`
    - `bypass`
- `molit8_parser_pvd.py`
  - PVD UDP 도구
  - subcommand:
    - `parse`
    - `bypass`

## Run

```bash
python tools/udp_debug/molit8_parser_rsa.py parse
python tools/udp_debug/molit8_parser_rsa.py record --log-dir logs
python tools/udp_debug/molit8_parser_rsa.py bypass --target 127.0.0.1:50002

python tools/udp_debug/molit8_parser_pvd.py parse
python tools/udp_debug/molit8_parser_pvd.py bypass --target 127.0.0.1:50001
```

## Notes

- 현재 GUI/CLI 예제 프로그램과 직접 연결되지 않습니다.
- receiver/panel 구조로 통합하지 않고 standalone 분석용 스크립트로만 보관합니다.
- RSA 쪽은 원래 아래 4개 파일의 기능을 통합했습니다.
  - `molit8_parser_rsa.py`
  - `molit8_parser_rsa_record.py`
  - `bypass_rsa.py`
