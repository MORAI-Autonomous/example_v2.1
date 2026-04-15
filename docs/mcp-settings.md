# MCP 설정 가이드

Claude Code에서 MCP(Model Context Protocol) 서버를 활성화하면 추가 기능을 사용할 수 있지만,
불필요한 MCP는 컨텍스트 낭비와 응답 지연을 유발한다.

## 이 프로젝트에서 필요한 MCP

| MCP | 용도 | 비고 |
|-----|------|------|
| 없음 (기본 도구만 사용) | Read/Edit/Bash/Grep 등 내장 도구로 충분 | - |

## 비활성화 권장 MCP

| MCP | 이유 |
|-----|------|
| Slack | 이 프로젝트에서 사용 안 함 |
| Chrome / Browser | UI는 DearPyGUI, 웹 브라우저 불필요 |
| Jupyter / Notebook | .ipynb 파일 없음 |
| 기타 외부 서비스 | 프로젝트와 무관 |

## MCP 비활성화 방법

### 방법 1: claude_desktop_config.json 편집
`%APPDATA%\Claude\claude_desktop_config.json` (Windows) 또는
`~/Library/Application Support/Claude/claude_desktop_config.json` (macOS):

```json
{
  "mcpServers": {
    "slack": { "disabled": true }
  }
}
```

### 방법 2: CLAUDE.md에 사용 안 함 명시
```markdown
## MCP
이 프로젝트는 MCP 서버를 사용하지 않는다.
외부 API 연동 없이 로컬 파일과 TCP/UDP 소켓만 사용한다.
```

### 방법 3: 프로젝트별 .claude/settings.json
```json
{
  "enabledMcpServers": []
}
```

## 토큰 절약 효과

MCP 서버를 비활성화하면:
- 시스템 프롬프트에 MCP 도구 스키마가 포함되지 않음 → 입력 토큰 절감
- 불필요한 tool_use 시도 없음 → 처리 속도 향상

## 참고

MCP 설정은 전역(claude_desktop_config.json)과 프로젝트별(.claude/settings.json)로 구분된다.
프로젝트별 설정이 전역 설정을 오버라이드한다.
