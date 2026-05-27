#!/bin/bash
# Monitor logs from all running pseudo-gt test containers in real-time

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

COLORS=($BLUE $CYAN $YELLOW $GREEN $RED)

show_usage() {
    cat << EOF
Usage: tail_all_tests.sh [OPTIONS]

Monitor logs from all running pseudo-gt containers.

OPTIONS:
  -f, --follow      Follow logs in real-time (default)
  -n, --lines N     Show last N lines (default: 20)
  -a, --all         Show all logs (no line limit)
  -s, --summary     Show current progress summary
  -h, --help        Show this help message

EXAMPLES:
  tail_all_tests.sh              # Show last 20 lines from all containers
  tail_all_tests.sh -n 50        # Show last 50 lines
  tail_all_tests.sh -f           # Follow logs in real-time
  tail_all_tests.sh -s           # Show progress summary
EOF
}

follow_logs=false
lines=20
summary_mode=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--follow)
            follow_logs=true
            shift
            ;;
        -n|--lines)
            lines="$2"
            shift 2
            ;;
        -a|--all)
            lines="all"
            shift
            ;;
        -s|--summary)
            summary_mode=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Find all running pseudo-gt containers
containers=$(podman ps --format "{{.Names}}" | grep "pseudo-gt" | sort)

if [ -z "$containers" ]; then
    echo "No running pseudo-gt containers found."
    exit 0
fi

container_count=$(echo "$containers" | wc -l)
echo -e "${GREEN}Found $container_count running container(s):${NC}"
echo "$containers" | nl

if [ "$summary_mode" = true ]; then
    echo ""
    echo -e "${CYAN}=== Progress Summary ===${NC}"
    i=0
    for container in $containers; do
        color=${COLORS[$((i % ${#COLORS[@]}))]};
        progress=$(podman logs "$container" 2>/dev/null | grep "\[progress\]" | tail -1 || echo "No progress yet")
        echo -e "${color}$container:${NC}"
        echo "  $progress"
        i=$((i + 1))
    done
    exit 0
fi

if [ "$follow_logs" = true ]; then
    echo ""
    echo -e "${CYAN}=== Following logs (Ctrl+C to stop) ===${NC}"
    echo ""

    # Start tail processes for each container
    pids=()
    for container in $containers; do
        color_code=""
        case ${COLORS[$(($(echo "$containers" | grep -n "^$container$" | cut -d: -f1) - 1))%${#COLORS[@]}]} in
            "$BLUE") color_code="\033[0;34m" ;;
            "$CYAN") color_code="\033[0;36m" ;;
            "$YELLOW") color_code="\033[1;33m" ;;
            "$GREEN") color_code="\033[0;32m" ;;
            "$RED") color_code="\033[0;31m" ;;
        esac

        # Stream logs with container name prefix
        (podman logs -f "$container" 2>/dev/null | sed "s/^/${color_code}[$container]${NC} /" || true) &
        pids+=($!)
    done

    # Wait for all background jobs
    trap "kill ${pids[@]} 2>/dev/null; exit" INT TERM
    wait
else
    echo ""
    echo -e "${CYAN}=== Last $lines lines from each container ===${NC}"
    echo ""

    i=0
    for container in $containers; do
        color=${COLORS[$((i % ${#COLORS[@]}))]};
        echo -e "${color}=== $container ===${NC}"
        if [ "$lines" = "all" ]; then
            podman logs "$container" 2>/dev/null | tail || true
        else
            podman logs "$container" 2>/dev/null | tail -n "$lines" || true
        fi
        echo ""
        i=$((i + 1))
    done
fi
