#!/usr/bin/env bash
# ============================================================================
#  flash.sh — STM32 펌웨어 빌드 → RPi 전송 → SWD 플래시 (검증 포함)
# ----------------------------------------------------------------------------
#  Mac 에서 ST-Link 가 안 잡히는 문제 때문에 **RPi 를 플래시 호스트로 쓴다.**
#  Pi 에 꽂힌 ST-Link 로 OpenOCD 가 SWD 로 굽고, 읽어서 대조까지 한다.
#
#  두 가지 모드
#    (기본)  Mac 에서 실행 → 빌드 · scp · 원격 플래시를 한 번에
#    -l      Pi 에서 직접 실행 → 이미 있는 elf 를 굽기만
#
#  ⚠️ 왜 드래그앤드롭(MSD)이 아니라 SWD 인가 = **검증** 때문이다.
#     NUCLEO 의 USB 드라이브에 .bin 을 cp 하면 거의 항상 성공을 반환하는데,
#     그건 데이터를 장치에 넘겼다는 뜻이지 플래시에 제대로 앉았다는 뜻이 아니다.
#     읽어서 대조하지 않으므로 부분 기록이 나도 모른다. 홈·엔코더·모터를
#     동시에 쫓는 상황에서 "펌웨어가 제대로 올라갔나"가 불확실하면 진단이
#     못 굴러간다. SWD 는 `** Verified OK **` 를 준다.
#
#  ⚠️ .elf 를 그대로 쓴다 — objcopy 단계가 없다. 옛 .bin 을 올려놓고
#     "코드를 고쳤는데 왜 그대로지" 하는 사고를 원천 차단한다.
#
#  사전 준비
#    Pi:  sudo apt install openocd
#    Mac: ssh-copy-id adts-pi     ← 한 번만. 안 하면 매번 비밀번호를 묻는다
# ==========================================================================*/
set -uo pipefail

HOST="adts-pi"
ELF=""
LOCAL=0
NO_BUILD=0
REMOTE_NAME="adts.elf"

usage() {
    cat <<EOF
사용법: $0 [옵션]

  -H <host>   RPi ssh 대상            (기본 $HOST)
  -e <경로>   플래시할 .elf           (기본 build/Debug/adts.elf)
  -l          로컬 모드 — 이 기계에서 바로 플래시 (Pi 에서 실행할 때)
  -n          빌드 건너뛰기
  -h          이 도움말

예)
  $0                       Mac 에서: 빌드 → 전송 → 플래시 → 검증
  $0 -n                    빌드 없이 지금 elf 그대로
  $0 -H pi@10.144.31.125   호스트 직접 지정
  $0 -l -e ~/adts.elf      Pi 에서 직접
EOF
}

while getopts ":H:e:lnh" opt; do
    case "$opt" in
        H) HOST=$OPTARG ;;
        e) ELF=$OPTARG ;;
        l) LOCAL=1 ;;
        n) NO_BUILD=1 ;;
        h) usage; exit 0 ;;
        \?) echo "알 수 없는 옵션: -$OPTARG" >&2; usage; exit 2 ;;
        :)  echo "-$OPTARG 는 값이 필요하다" >&2; exit 2 ;;
    esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ -n "$ELF" ] || ELF="$REPO/build/Debug/adts.elf"

say()  { echo "── $*"; }
die()  { echo "!! $*" >&2; exit 1; }

# 플랫폼마다 다르다. 전송 전후 대조에 쓴다.
md5of() {
    if   command -v md5sum >/dev/null; then md5sum "$1" | awk '{print $1}'
    elif command -v md5    >/dev/null; then md5 -q "$1"
    else echo "-"; fi
}

# OpenOCD 명령. ⚠️ `~` 를 쓰면 안 된다 — -c "..." 안의 문자열은 OpenOCD 에
# 그대로 전달되고 틸데 확장이 일어나지 않아 "couldn't open ~/adts.elf" 가 난다.
# $HOME 은 큰따옴표 안에서 셸이 치환하므로 동작한다. 여기서는 아예 절대경로를
# 만들어 넘겨 그 함정 자체를 없앤다.
openocd_cmd() {
    printf 'openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program %s verify reset exit"' "$1"
}

# ── 빌드 ────────────────────────────────────────────────────────────────────
if [ "$NO_BUILD" -eq 0 ] && [ "$LOCAL" -eq 0 ]; then
    if [ -d "$REPO/build/Debug" ]; then
        say "빌드"
        cmake --build "$REPO/build/Debug" -j8 2>&1 | grep -E "warning: #warning|error|RAM:|FLASH:|ninja: no work" || true
        # ⚠️ 파이프를 거쳤으므로 $? 는 grep 것이다. 실제 결과를 봐야 한다.
        [ "${PIPESTATUS[0]}" -eq 0 ] || die "빌드 실패"
    else
        echo "⚠ build/Debug 가 없다 — 빌드를 건너뛴다 (cmake --preset Debug 먼저)"
    fi
fi

[ -f "$ELF" ] || die "elf 가 없다: $ELF"
LOCAL_MD5="$(md5of "$ELF")"
say "$(basename "$ELF")  $(wc -c < "$ELF" | tr -d ' ') bytes  md5=$LOCAL_MD5"

# 어떤 커밋인지 남긴다. 나중에 "보드에 뭐가 올라가 있지" 를 되짚을 때 이 한 줄이 전부다.
if git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1; then
    DIRTY=""
    git -C "$REPO" diff --quiet 2>/dev/null || DIRTY=" +미커밋"
    say "소스: $(git -C "$REPO" rev-parse --short HEAD) ($(git -C "$REPO" branch --show-current))$DIRTY"
fi

# ── 로컬 모드 ───────────────────────────────────────────────────────────────
if [ "$LOCAL" -eq 1 ]; then
    command -v openocd >/dev/null || die "openocd 가 없다: sudo apt install openocd"
    ABS="$(cd "$(dirname "$ELF")" && pwd)/$(basename "$ELF")"
    say "플래시 (SWD)"
    OUT=$(eval "$(openocd_cmd "$ABS")" 2>&1); RC=$?
    echo "$OUT"
    if echo "$OUT" | grep -q "Verified OK"; then
        echo; say "✅ 완료 — Verified OK"
        exit 0
    fi
    die "검증 실패 (rc=$RC) — 위 출력 확인"
fi

# ── 원격 모드 ───────────────────────────────────────────────────────────────
say "대상: $HOST"
if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$HOST" true 2>/dev/null; then
    cat >&2 <<EOF
!! $HOST 에 키 인증으로 접속할 수 없다.

   터미널에서 한 번만 실행할 것 (비밀번호 1회):
       ssh-copy-id $HOST

   (Claude Code 등 TTY 없는 환경에서는 비밀번호 입력이 안 된다)
EOF
    exit 1
fi

say "전송 → $HOST:~/$REMOTE_NAME"
scp -q "$ELF" "$HOST:$REMOTE_NAME" || die "scp 실패"

# ⚠️ 전송 무결성을 반드시 대조한다. 파일이 깨진 채 구우면 OpenOCD 는
#    "그 깨진 내용" 으로 검증을 통과시킨다 — verify 는 elf 와 플래시를
#    비교할 뿐 elf 자체가 맞는지는 모른다.
REMOTE_MD5=$(ssh "$HOST" "md5sum \$HOME/$REMOTE_NAME 2>/dev/null | awk '{print \$1}'")
if [ -n "$LOCAL_MD5" ] && [ "$LOCAL_MD5" != "-" ] && [ "$LOCAL_MD5" != "$REMOTE_MD5" ]; then
    die "md5 불일치 — 전송 손상
       로컬  $LOCAL_MD5
       원격  $REMOTE_MD5"
fi
say "md5 일치"

say "플래시 (SWD) — Verified OK 를 기다린다"
OUT=$(ssh "$HOST" "command -v openocd >/dev/null || { echo 'NO_OPENOCD'; exit 90; };
                   $(openocd_cmd "\$HOME/$REMOTE_NAME")" 2>&1)
RC=$?
echo "$OUT"

if echo "$OUT" | grep -q "NO_OPENOCD"; then
    die "$HOST 에 openocd 가 없다:  ssh $HOST 'sudo apt install -y openocd'"
fi
if echo "$OUT" | grep -q "Verified OK"; then
    echo
    say "✅ 완료 — Verified OK"
    cat <<EOF

── 바로 확인 ────────────────────────────────────────────────────────────────
  ssh $HOST 'cd ~/final_project/driver && ./turret_test home && sleep 15 && ./turret_test state'

  last_err  0 + homed=1 = 성공
            3 = 양축 엔코더 판독 실패
            4 = 팬 엔코더만 실패   (브링업 프로브 — 범위오류 아님)
            6 = 틸트 엔코더만 실패 (브링업 프로브 — 라이다 무관)
EOF
    exit 0
fi

die "검증 실패 (rc=$RC) — 위 출력 확인.
   ST-Link 가 안 잡히면:  ssh $HOST 'lsusb | grep -i st'"
