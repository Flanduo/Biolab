#!/bin/bash
#
# OpenArm 电机初始化配置脚本（步骤可拆分执行）
#
# 子命令（任选其一；不写子命令时等价于 all，兼容旧用法）：
#   can20     仅将 CAN 配成 2.0（1Mbps），用于写电机波特率前
#   baudrate  仅写电机波特率 5Mbps（Flash）
#   zero      仅设零位（需确认）
#   canfd     仅将 CAN 配成 CAN-FD（与电机 5M 数据段匹配）
#   verify    仅发使能帧做通信验证（可选交互）
#   all       按顺序执行以上全部（可用 --skip-zero / --skip-verify 跳过部分）
#
# 用法示例：
#   ./init_motors.sh can20
#   ./init_motors.sh baudrate --can can0 --id 4
#   ./init_motors.sh zero --can can1 --id 4
#   ./init_motors.sh canfd
#   ./init_motors.sh verify
#   ./init_motors.sh all --skip-zero
#
# 单电机须同时指定：--can can0|can1 --id 1-8
#

# 确保使用 bash 运行
if [ -z "$BASH_VERSION" ]; then
    if command -v bash >/dev/null 2>&1; then
        exec bash "$0" "$@"
    else
        echo "错误: 此脚本需要使用 bash 运行"
        exit 1
    fi
fi

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

WS_DIR="${ROS2_WS:-$HOME/ros2_ws}"
SETUP_DIR="$WS_DIR/src/openarm_can/setup"

print_info() {
    echo -e "${BLUE}[信息]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[成功]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[警告]${NC} $1"
}

print_error() {
    echo -e "${RED}[错误]${NC} $1"
}

usage() {
    echo "OpenArm 电机初始化（分步执行）"
    echo ""
    echo "用法: $0 [子命令] [选项]"
    echo ""
    echo "子命令:"
    echo "  can20     配置 CAN 为 2.0（写波特率前必须）"
    echo "  baudrate  写入电机波特率 5Mbps 并 --flash"
    echo "  zero      设置电机零位"
    echo "  canfd     配置 CAN 为 CAN-FD（正常运行用）"
    echo "  verify    发送使能帧验证通信"
    echo "  all       依次执行 can20 → baudrate → zero → canfd → verify"
    echo "  help      显示本帮助"
    echo ""
    echo "不写子命令时，默认执行 all（与旧版兼容）。"
    echo ""
    echo "选项:"
    echo "  --skip-zero     仅对 all：跳过 zero"
    echo "  --skip-verify   仅对 all：跳过 verify"
    echo "  --can <iface>   can0 或 can1（须与 --id 同时使用）"
    echo "  --id <1-8>      单电机模式"
    echo ""
    echo "示例:"
    echo "  $0 can20"
    echo "  $0 baudrate --can can0 --id 4"
    echo "  $0 zero --can can1 --id 4"
    echo "  $0 all --skip-zero"
}

SKIP_ZERO=false
SKIP_VERIFY=false
TARGET_CAN=""
TARGET_ID=""
STEP="all"

if [ $# -ge 1 ]; then
    case "$1" in
        can20|baudrate|zero|canfd|verify|all|help)
            STEP="$1"
            shift
            ;;
    esac
fi

while [ $# -gt 0 ]; do
    case $1 in
        --skip-zero)
            SKIP_ZERO=true
            shift
            ;;
        --skip-verify)
            SKIP_VERIFY=true
            shift
            ;;
        --can)
            if [ $# -lt 2 ]; then
                print_error "--can 需要参数（can0 或 can1）"
                exit 1
            fi
            TARGET_CAN="$2"
            shift 2
            ;;
        --id)
            if [ $# -lt 2 ]; then
                print_error "--id 需要参数（1-8）"
                exit 1
            fi
            TARGET_ID="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            usage
            exit 1
            ;;
    esac
done

if [ "$STEP" = "help" ]; then
    usage
    exit 0
fi

if [ -n "$TARGET_CAN" ] && [ "$TARGET_CAN" != "can0" ] && [ "$TARGET_CAN" != "can1" ]; then
    print_error "--can 仅支持 can0 或 can1"
    exit 1
fi

if [ -n "$TARGET_ID" ] && ! [[ "$TARGET_ID" =~ ^[0-9]+$ ]]; then
    print_error "--id 必须是数字（1-8）"
    exit 1
fi

if [ -n "$TARGET_ID" ] && { [ "$TARGET_ID" -lt 1 ] || [ "$TARGET_ID" -gt 8 ]; }; then
    print_error "--id 仅支持 1-8"
    exit 1
fi

if { [ -n "$TARGET_CAN" ] && [ -z "$TARGET_ID" ]; } || { [ -z "$TARGET_CAN" ] && [ -n "$TARGET_ID" ]; }; then
    print_error "单电机模式需同时指定 --can 和 --id"
    exit 1
fi

SINGLE_MODE=false
if [ -n "$TARGET_CAN" ] && [ -n "$TARGET_ID" ]; then
    SINGLE_MODE=true
fi

if [ ! -d "$WS_DIR" ]; then
    print_error "工作空间目录不存在: $WS_DIR"
    exit 1
fi

if [ ! -d "$SETUP_DIR" ]; then
    print_error "找不到 setup 目录: $SETUP_DIR"
    exit 1
fi

if [ ! -f "$SETUP_DIR/configure_socketcan.sh" ]; then
    print_error "找不到 configure_socketcan.sh 脚本"
    exit 1
fi

if [ ! -f "$SETUP_DIR/change_baudrate.py" ]; then
    print_error "找不到 change_baudrate.py 脚本"
    exit 1
fi

if [ ! -f "$SETUP_DIR/set_zero.sh" ]; then
    print_error "找不到 set_zero.sh 脚本"
    exit 1
fi

check_can_interface() {
    local iface=$1
    if ip link show "$iface" &>/dev/null 2>&1; then
        local state
        state=$(ip link show "$iface" 2>/dev/null | grep -oE 'state [A-Z]+' | awk '{print $2}' || echo "UNKNOWN")
        if [ "$state" = "DOWN" ]; then
            print_info "CAN 接口 $iface 存在但处于 DOWN 状态，配置脚本将自动激活"
        elif [ "$state" = "UP" ]; then
            print_info "CAN 接口 $iface 已激活"
        fi
        return 0
    else
        if [ -d "/sys/class/net/$iface" ]; then
            print_info "CAN 接口 $iface 存在（通过 /sys/class/net 检测）"
            return 0
        else
            print_error "CAN 接口 $iface 不存在"
            print_info "可用接口列表："
            ip link show | grep -E "^[0-9]+:" | awk '{print $2}' | sed 's/:$//' | grep -E "^can" || echo "  未找到 CAN 接口"
            return 1
        fi
    fi
}

step_can20() {
    print_info "[can20] 配置 CAN 接口为 CAN 2.0 模式（1Mbps）"
    print_warning "写入电机波特率前必须使用 CAN 2.0 模式"
    if [ "$SINGLE_MODE" = true ]; then
        if ! check_can_interface "$TARGET_CAN"; then
            exit 1
        fi
        if sudo "$SETUP_DIR/configure_socketcan.sh" "$TARGET_CAN"; then
            print_success "$TARGET_CAN CAN 2.0 配置完成"
        else
            print_error "$TARGET_CAN 配置失败"
            exit 1
        fi
    else
        for iface in can0 can1; do
            if ! check_can_interface "$iface"; then
                exit 1
            fi
        done
        for iface in can0 can1; do
            print_info "配置 $iface ..."
            if sudo "$SETUP_DIR/configure_socketcan.sh" "$iface"; then
                print_success "$iface 配置完成"
            else
                print_error "$iface 配置失败"
                exit 1
            fi
        done
    fi
}

step_baudrate() {
    print_info "[baudrate] 设置电机波特率为 5Mbps 并写入 Flash"
    print_warning "电机参数有 10000 次写入限制，请谨慎操作"
    cd "$SETUP_DIR" || exit 1
    if [ "$SINGLE_MODE" = true ]; then
        print_info "  $TARGET_CAN 电机 ID $TARGET_ID ..."
        if python3 change_baudrate.py --baudrate 5000000 --canid "$TARGET_ID" --socketcan "$TARGET_CAN" --flash; then
            print_success "$TARGET_CAN ID $TARGET_ID 波特率设置完成"
        else
            print_error "$TARGET_CAN ID $TARGET_ID 波特率设置失败"
            exit 1
        fi
    else
        for iface in can0 can1; do
            print_info "总线 $iface ，ID 1-8 ..."
            ok=true
            for motor_id in 1 2 3 4 5 6 7 8; do
                print_info "  ID $motor_id ..."
                if python3 change_baudrate.py --baudrate 5000000 --canid "$motor_id" --socketcan "$iface" --flash; then
                    print_success "  ID $motor_id 完成"
                else
                    print_error "  ID $motor_id 失败"
                    ok=false
                fi
                sleep 0.5
            done
            if [ "$ok" = true ]; then
                print_success "$iface 全部完成"
            else
                print_warning "$iface 部分失败，请检查连接"
            fi
        done
    fi
    cd "$WS_DIR" || exit 1
}

step_zero() {
    print_info "[zero] 设置电机零位"
    print_warning "执行前请将电机摆到机械零位"
    echo ""
    read -p "确认已定位到零位？(y/N): " -n 1 -r
    echo ""
    cd "$SETUP_DIR" || exit 1
    case "$REPLY" in
        [Yy]*)
            if [ "$SINGLE_MODE" = true ]; then
                hid=$(printf "%03d" "$TARGET_ID")
                if ./set_zero.sh "$TARGET_CAN" "$hid"; then
                    print_success "$TARGET_CAN ID $hid 零位完成"
                else
                    print_error "零位失败"
                    exit 1
                fi
            else
                if ./set_zero.sh can0 --all; then
                    print_success "can0 零位完成"
                else
                    print_error "can0 零位失败"
                    exit 1
                fi
                if ./set_zero.sh can1 --all; then
                    print_success "can1 零位完成"
                else
                    print_error "can1 零位失败"
                    exit 1
                fi
            fi
            ;;
        *)
            print_warning "已跳过零位"
            print_info "可手动：cd $SETUP_DIR && ./set_zero.sh <can> <001-008 或 --all>"
            ;;
    esac
    cd "$WS_DIR" || exit 1
}

step_canfd() {
    print_info "[canfd] 配置 CAN 为 CAN-FD（1M / 5M 数据）"
    if [ "$SINGLE_MODE" = true ]; then
        if sudo "$SETUP_DIR/configure_socketcan.sh" "$TARGET_CAN" -fd; then
            print_success "$TARGET_CAN CAN-FD 完成"
        else
            print_error "$TARGET_CAN CAN-FD 失败"
            exit 1
        fi
    else
        for iface in can0 can1; do
            if sudo "$SETUP_DIR/configure_socketcan.sh" "$iface" -fd; then
                print_success "$iface CAN-FD 完成"
            else
                print_error "$iface CAN-FD 失败"
                exit 1
            fi
        done
    fi
}

step_verify() {
    print_info "[verify] 通信验证（建议另开终端 candump）"
    echo ""
    read -p "是否发送使能帧测试？(y/N): " -n 1 -r
    echo ""
    if echo "$REPLY" | grep -qiE '^[Yy]'; then
        if [ "$SINGLE_MODE" = true ]; then
            cid=$(printf "%03d" "$TARGET_ID")
            if cansend "$TARGET_CAN" "${cid}#FFFFFFFFFFFFFFFC" 2>/dev/null; then
                print_success "已发 $TARGET_CAN ID $TARGET_ID 使能"
            else
                print_warning "发送失败"
            fi
        else
            for iface in can0 can1; do
                for i in 1 2 3 4 5 6 7 8; do
                    cid=$(printf "%03d" "$i")
                    if cansend "$iface" "${cid}#FFFFFFFFFFFFFFFC" 2>/dev/null; then
                        echo "  ✓ $iface ID $i"
                    else
                        print_warning "  $iface ID $i 发送失败"
                    fi
                done
            done
        fi
        print_info "若 candump 有回包则通信正常"
    else
        print_info "已跳过；可手动 candump / cansend"
    fi
}

step_all() {
    print_info "执行全流程：can20 → baudrate → zero → canfd → verify"
    echo ""
    step_can20
    echo ""
    step_baudrate
    echo ""
    if [ "$SKIP_ZERO" = false ]; then
        step_zero
    else
        print_info "已跳过 zero（--skip-zero）"
    fi
    echo ""
    step_canfd
    echo ""
    if [ "$SKIP_VERIFY" = false ]; then
        step_verify
    else
        print_info "已跳过 verify（--skip-verify）"
    fi
}

print_info "工作空间: $WS_DIR"
if [ "$SINGLE_MODE" = true ]; then
    print_info "单电机: $TARGET_CAN ID $TARGET_ID"
fi
echo ""

case "$STEP" in
    can20)   step_can20 ;;
    baudrate) step_baudrate ;;
    zero)    step_zero ;;
    canfd)   step_canfd ;;
    verify)  step_verify ;;
    all)     step_all ;;
    *)
        print_error "未知子命令: $STEP"
        usage
        exit 1
        ;;
esac

print_success "步骤 [$STEP] 执行结束。"
