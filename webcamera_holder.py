"""
ELP USB4KHDR01 (38.17 x 38.17 mm, t=1.64 mm) 用 Web カメラホルダ
---------------------------------------------------------------------
参考: "ELP Camera Housing (USBFHD04H)" by Azure (BY-NC) の構成を踏襲し、
基板サイズ 32mm -> 38.17mm、裏面縦型 4P コネクタ (配線が真後ろへ出る) に合わせて
作り直したもの。部品は 4 種類:

  1. housing : 本体 (前面レンズ開口 + ボス 4 本 + 通気スロット + ヒンジ脚)
  2. cover   : 裏蓋 (スナップ嵌合、コネクタ直上にケーブル切り欠き、
               基板の四隅を押さえる柱 4 本 → ねじ無しで基板を固定)
  3. mount   : L 形ブラケット (上面パッド + 前面アーム、フック無し)。外側 2 面にテープ貼り付け用の
               プレートを持ち、モニタの上辺や側辺にテープ / 両面テープで留められる
  4. pin     : ヒンジピン (D 形断面。脚とマウントの D 穴に通すと回転が止まり、
               角度が FIXED_TILT_DEG に固定される)
               太さ違いを PIN_SIZES 分だけ出力し、合うものを選ぶ

ねじ無しで組める構成。M2 ねじが手に入った場合は、本体のボス穴をそのまま使って
基板をねじ止めすることもできる。

座標系 (housing):
  原点 = 基板中心、+Z = 基板の裏側 (カバー側)、-Z = レンズ側、+Y = 上 (コネクタ側)
  housing はレンズ面が Z=0 にあり、そのままの向きで印刷できる (レンズ面がベッド)。
  mount も housing 座標系の「組み立て位置」で作り、印刷時に寝かせる。
  このため脚の D 穴とマウントの D 穴は同じ向きで切られ、ピンを通せば角度が揃う。

Bambu Lab A1 mini / PLA, 0.2mm レイヤ, 壁 2-3 本を想定。
"""

import math
import os
from pathlib import Path

from build123d import *  # noqa: F401,F403

try:
    from ocp_vscode import show
except ImportError:  # viewer が無い環境でも実行できるように
    show = None

# ---------------------------------------------------------------------------
# パラメータ (単位 mm)
# ---------------------------------------------------------------------------
# --- 基板 (実測値) ---
BOARD = 38.17            # 基板 縦横
BOARD_T = 1.64           # 基板 厚さ
BOARD_CLR = 0.40         # 基板周囲の片側クリアランス
HOLE_PITCH = 34.0        # 取付穴ピッチ (写真から推定、要確認)
SCREW_HOLE_D = 1.8       # M2 タッピング下穴
BOSS_D = 6.0             # M2 ボス外径

LENS_OPEN = 22.5         # 前面レンズ開口 (正方形)。ホルダ台座 約20mm角 + 余裕
LENS_OPEN_R = 1.0        # 開口の角 R

# --- コネクタ (基板裏面の縦型 4P 2.0mm) ---
CONN_X = 0.0             # 基板中心からの X オフセット (前面から見て右が +)
CONN_FROM_EDGE = 6.0     # 基板上辺 (+Y 端) からコネクタ中心までの距離
CONN_W = 10.5            # コネクタ (プラグ込み) X 方向寸法
CONN_D = 5.0             # コネクタ (プラグ込み) Y 方向寸法
CONN_CLR = 1.0           # 切り欠きの片側余裕
CABLE_WALL_SLOT = True   # 上壁にもケーブル用スリットを入れる (上方向にも逃がせる)
CABLE_SLOT_W = 5.0       # 上壁スリット幅 (ケーブル径 + 余裕)
CABLE_SLOT_D = 6.0       # 上壁スリット深さ (背面端から)

# --- 本体 ---
WALL = 2.0               # 壁厚 (1.5 -> 2.0: スナップ受け溝を深くできるように)
FRONT_T = 1.0            # 前面板厚
STANDOFF = 7.0           # 前面板内面 -> 基板表面 (ボス高さ)
REAR = 10.0              # 基板裏面 -> 本体背面端 (コネクタ + プラグ 約 8mm)
CORNER_R = 2.0           # 外形 縦エッジ R
FRONT_CHAMFER = 1.0      # 前面外周の面取り
LEAD_IN = 0.6            # 背面開口の内側エッジ面取り (裏蓋リップの入口)

INNER = BOARD + 2 * BOARD_CLR            # 内寸 38.97
OUTER = INNER + 2 * WALL                 # 外寸 42.97
DEPTH = FRONT_T + STANDOFF + BOARD_T + REAR  # 本体奥行 19.64
BOARD_Z = FRONT_T + STANDOFF             # 基板表面の Z
HOLE_OFF = HOLE_PITCH / 2

# --- 通気スロット (縦長の長穴。印刷時は壁が垂直なので縦スロットはブリッジ不要) ---
VENTS = True
VENT_SLOT_W = 1.8        # スロット幅
VENT_PITCH = 3.5         # スロット間隔
VENT_SIDE_XS = [i * VENT_PITCH for i in range(-3, 4)]        # 左右壁: 7 本 (|Y| <= 11.4、ボスと干渉しない)
VENT_TOP_XS = [s * x for s in (-1, 1) for x in (4.5, 8.0, 11.5)]  # 上壁: 中央のケーブルスリットを避けて 6 本

# --- ヒンジ脚 (本体下側、マウントを挟む) ---
LEG_T = 4.0              # 脚厚 (X)
LEG_GAP = 7.0            # 脚の内側間隔 (マウント厚 6.5 + 0.5)
LEG_LEN = 14.4           # 本体下壁からの突き出し長
LEG_H = 11.3             # 脚の奥行 (前面 Z=0 から)
PIVOT_FROM_BOX = 8.7     # 本体下壁 -> ピボット中心
PIVOT_Z = 5.7            # ピボット中心の Z
PIVOT_Y = -OUTER / 2 - PIVOT_FROM_BOX
LEG_HOLE_D = 4.2         # ピン穴径 (D 形。平面側は印刷時に上になるのでブリッジが短くて済む)
HOLE_FLAT_FROM_AXIS = 1.2  # 穴の平面 (D の直線部) の軸からの距離。平面は +Z 側

# --- 裏蓋 ---
COVER_T = 2.5            # 板厚 (2.0 -> 2.5)
LIP_H = 3.5              # 嵌合リップ高さ (1.5 -> 3.5: 溝の上に受け棚を残すため)
LIP_T = 2.0              # リップ厚
LIP_CLR = 0.15           # リップ外面と本体内壁の片側隙間
SNAP_LEN = 12.0          # スナップ突起の長さ (左右壁、Y 方向)
SNAP_PROUD = 0.8         # 突起の出っ張り (0.45 -> 0.8)
SNAP_H = 1.6             # 突起の高さ (Z)。下側 SNAP_PROUD 分は 45° の導入テーパ
GROOVE_DEPTH = 0.9       # 本体内壁の受け溝深さ (0.5 -> 0.9)
# 溝の上に残る受け棚の厚さ。旧設計は 0.3 (ほぼ 1 層) しか無く、これが抜けやすさの原因
LEDGE_T = LIP_H - 0.1 - (SNAP_H + 0.3)
assert LEDGE_T >= 1.0, f"受け棚が薄すぎる: {LEDGE_T:.2f}"
assert SNAP_PROUD <= GROOVE_DEPTH + LIP_CLR, "突起が溝に収まらない"

# --- 基板押さえ (裏蓋の柱。ねじの代わりに基板をボスとの間に挟む) ---
BOARD_PRESS = True       # 柱を付ける
PRESS_POST_D = 3.5       # 柱の直径 (取付穴まわりのランドを押す)
PRESS_INTERF = 0.15      # 押し込み量 (スナップの遊び 0.2mm 以内)
PRESS_PIN = False        # 柱の先に基板穴へ入る位置決めピンを付ける
PRESS_PIN_D = 1.6        #   (穴ピッチ HOLE_PITCH が推定値なので既定は無効)
PRESS_PIN_LEN = 2.5

LIP_OUT = INNER - 2 * LIP_CLR
LIP_IN = LIP_OUT - 2 * LIP_T

# --- マウント (L 形。housing 座標系の組み立て位置で作る) ---
#   上面パッドがモニタ上面に載り、前面アームがモニタ前面に沿って下りる。
#   ピボットボスは前面アームの前側に張り出し、本体の脚がこれを挟む。
MOUNT_T = 6.5            # 厚さ (X)。脚の間隔 7.0 - 0.5
ARM_W = 5.0              # 前面アームの幅 (Z)
PAD_T = 5.0              # 上面パッドの厚さ (Y)
PAD_LEN = 28.0           # モニタ前面からパッド後端までの長さ
ARM_BELOW = 30.0         # ピボット中心からアーム下端までの長さ
MOUNT_BOSS_R = 5.5       # ピボットボス半径 (脚の奥行 11.3 に収まる)
MONITOR_FACE_DZ = MOUNT_BOSS_R + 2.0   # ピボット中心 -> モニタ前面 (= アーム背面) の距離 (+Z)。俯角 30° でも脚がモニタに当たらない値
PAD_TOP_CLR = 1.0                      # パッド上面と本体下壁の隙間 (FIXED_TILT_DEG=0 のとき。俯角を付けるほど隙間は広がる)
PAD_TOP_DY = PIVOT_FROM_BOX - PAD_TOP_CLR   # ピボット中心 -> パッド上面 (+Y) = 7.7
MONITOR_TOP_DY = PAD_TOP_DY - PAD_T         # ピボット中心 -> モニタ上面 (+Y) = 2.7
FIXED_TILT_DEG = 27.0    # レンズの俯角 (+ で下向き)。D 穴の向きで決まる固定角。初版の写真の姿勢から読み取った値
MOUNT_HOLE_D = LEG_HOLE_D  # ピン穴。脚と同じ D 形

# --- 貼り付け用プレート (テープ / 両面テープ / 面ファスナー用。L の外側 2 面を広い面にする) ---
#   寝かせて印刷するため、プレートはバーの片側面と面一にして反対側へ張り出す
#   (両側に張り出すとバーとボスが浮いてサポートが要る)。
PLATES = True
PLATE_W = 26.0           # プレート幅 (X)。バー 6.5 を含む
PLATE_T = 2.0            # プレート厚
PLATE_R = 3.0            # プレート角 R
PLATE_SIDE = 1           # 張り出す向き (+1: 前面から見て右、-1: 左)
PAD_PLATE_V0 = LEG_H - PIVOT_Z + 0.4   # パッド側プレートの前端 (脚の背面 5.6 より後ろ) = 6.0
ARM_PLATE_U0 = -9.0      # アーム側プレートの上端 (俯角 35° まで脚が当たらない)
WEB_X0 = LEG_GAP / 2 + LEG_T + 1.0   # 脚の外側面 7.5 + 1.0。ここから外側は L の角まで板を繋ぐ

# --- ヒンジピン (D 形断面) ---
PIN_SIZES = (3.9, 4.0, 4.1)   # 軸径の候補。頭の点の数 1,2,3 で区別 (4.0 が実績値)
PIN_LEN = 2 * LEG_T + LEG_GAP  # 軸長 15 (両脚を貫通し、反対側で面一)
PIN_HEAD_D = 7.0              # 頭の直径
PIN_HEAD_T = 1.6              # 頭の厚さ
PIN_FLAT_FROM_AXIS = 1.1      # 軸の平面 (D の直線部) の軸中心からの距離。穴の 1.2 に対し 0.1 の遊び
PIN_TIP_CHAMFER = 0.4         # 先端の面取り (差し込みやすく)

OUT_DIR = Path(__file__).with_name("output")


# ---------------------------------------------------------------------------
# 補助
# ---------------------------------------------------------------------------
def d_profile(r: float, flat_from_axis: float, center=(0.0, 0.0)) -> None:
    """D 形断面 (円 + スケッチローカル +y 側を平面でカット)。BuildSketch 内で呼ぶ。"""
    cx, cy = center
    with Locations((cx, cy)):
        Circle(r)
    with Locations((cx, cy + flat_from_axis + r)):
        Rectangle(2 * r + 1.0, 2 * r, mode=Mode.SUBTRACT)


def vent_cutter(xs, height: float, width: float = VENT_SLOT_W) -> Part:
    """縦長スロット列のカッター。ローカル XY に作り (Y がスロット長手)、
    Z 方向に壁を貫通する厚みで押し出す。"""
    with BuildSketch() as sk:
        with Locations(*[(x, 0) for x in xs]):
            SlotCenterToCenter(height - width, width, rotation=90)
    with BuildPart() as cutter:
        add(sk.sketch)
        extrude(amount=WALL + 0.6, both=True)
    return cutter.part


def place(part: Part, x: float, y: float) -> Part:
    """部品の XY 中心を (x, y) に、底面を Z=0 に移す (3MF の配置用)。"""
    bb = part.bounding_box()
    c = bb.center()
    return part.moved(Location((x - c.X, y - c.Y, -bb.min.Z)))


# ---------------------------------------------------------------------------
# 1. 本体 housing
# ---------------------------------------------------------------------------
def build_housing() -> Part:
    with BuildPart() as hp:
        # 外形ボックス (前面 Z=0, 背面 Z=DEPTH)
        Box(OUTER, OUTER, DEPTH, align=(Align.CENTER, Align.CENTER, Align.MIN))
        fillet(hp.edges().filter_by(Axis.Z), CORNER_R)

        # ヒンジ脚
        for sx in (-1, 1):
            with BuildPart() as leg:
                with BuildSketch(Plane.YZ) as lp:
                    # Plane.YZ: ローカル x = 全体 Y, ローカル y = 全体 Z
                    Rectangle(LEG_LEN + 1.0, LEG_H, align=(Align.MAX, Align.MIN))
                    far_top = lp.vertices().sort_by(Axis.X)[:2].sort_by(Axis.Y)[-1]
                    fillet(far_top, 3.0)
                extrude(amount=LEG_T / 2, both=True)
                try:
                    chamfer(leg.edges().filter_by(Axis.Z).group_by(Axis.Y)[0], 0.8)
                except Exception:
                    pass
            add(leg.part.moved(Location((sx * (LEG_GAP / 2 + LEG_T / 2), -OUTER / 2 + 1.0, 0))))

        # 前面外周の面取り
        try:
            chamfer(hp.faces().sort_by(Axis.Z)[0].edges(), FRONT_CHAMFER)
        except Exception:
            pass

        # 内部キャビティ (基板が入る空間)
        with BuildSketch(Plane.XY.offset(FRONT_T)):
            RectangleRounded(INNER, INNER, 0.8)
        extrude(amount=DEPTH, mode=Mode.SUBTRACT)

        # 背面開口の内側エッジ面取り (裏蓋リップの導入)
        try:
            back = hp.faces().sort_by(Axis.Z)[-1]
            chamfer(back.inner_wires()[0].edges(), LEAD_IN)
        except Exception as e:
            print("lead-in chamfer skipped:", e)

        # M2 ボス (前面板内面から STANDOFF の高さ)
        with BuildSketch(Plane.XY.offset(FRONT_T)):
            with Locations((HOLE_OFF, HOLE_OFF), (-HOLE_OFF, HOLE_OFF),
                           (HOLE_OFF, -HOLE_OFF), (-HOLE_OFF, -HOLE_OFF)):
                Circle(BOSS_D / 2)
        extrude(amount=STANDOFF)
        # ボスのタッピング穴 (前面板は貫通させない)
        with BuildSketch(Plane.XY.offset(BOARD_Z)):
            with Locations((HOLE_OFF, HOLE_OFF), (-HOLE_OFF, HOLE_OFF),
                           (HOLE_OFF, -HOLE_OFF), (-HOLE_OFF, -HOLE_OFF)):
                Circle(SCREW_HOLE_D / 2)
        extrude(amount=-(STANDOFF - 1.0), mode=Mode.SUBTRACT)

        # レンズ開口
        with BuildSketch(Plane.XY):
            RectangleRounded(LENS_OPEN, LENS_OPEN, LENS_OPEN_R)
        extrude(amount=FRONT_T + 0.1, both=True, mode=Mode.SUBTRACT)

        # ピボット穴 (両脚を X 方向に貫通、D 形。平面は +Z 側 = 印刷時の上側)
        with BuildSketch(Plane.YZ.offset(-OUTER / 2)):
            d_profile(LEG_HOLE_D / 2, HOLE_FLAT_FROM_AXIS, (PIVOT_Y, PIVOT_Z))
        extrude(amount=OUTER, mode=Mode.SUBTRACT)

        # 裏蓋スナップ受け溝 (左右内壁)。溝の上に LEDGE_T の受け棚が残る
        groove_zc = DEPTH - LIP_H + 0.1 + (SNAP_H + 0.3) / 2
        for sx in (-1, 1):
            with Locations((sx * (INNER / 2 + GROOVE_DEPTH / 2 - 0.01), 0, groove_zc)):
                Box(GROOVE_DEPTH + 0.02, SNAP_LEN + 1.0, SNAP_H + 0.3, mode=Mode.SUBTRACT)

        # 上壁ケーブルスリット
        if CABLE_WALL_SLOT:
            with Locations((CONN_X, OUTER / 2 - WALL / 2, DEPTH - CABLE_SLOT_D / 2 + 0.5)):
                Box(CABLE_SLOT_W, WALL + 2.0, CABLE_SLOT_D + 1.0, mode=Mode.SUBTRACT)

        # 通気スロット (上壁 + 左右壁)
        if VENTS:
            z_lo = FRONT_T + 2.0
            z_hi = DEPTH - LIP_H - 0.5
            zc = (z_lo + z_hi) / 2
            h = z_hi - z_lo - 1.0
            # 上壁 (+Y): ローカル X->X, ローカル Y->Z
            top = vent_cutter(VENT_TOP_XS, h)
            add(top.rotate(Axis.X, 90).moved(Location((0, OUTER / 2, zc))), mode=Mode.SUBTRACT)
            # 左右壁 (±X): ローカル X->Y, ローカル Y->Z
            side = vent_cutter(VENT_SIDE_XS, h)
            side = side.rotate(Axis.X, 90).rotate(Axis.Z, 90)
            for sx in (-1, 1):
                add(side.moved(Location((sx * OUTER / 2, 0, zc))), mode=Mode.SUBTRACT)

    return hp.part


# ---------------------------------------------------------------------------
# 2. 裏蓋 cover (本体座標で作成: 板は Z=DEPTH..DEPTH+COVER_T、リップは本体内へ。印刷用に後で裏返す)
# ---------------------------------------------------------------------------
def build_cover() -> Part:
    notch_w = CONN_W + 2 * CONN_CLR
    notch_y0 = BOARD / 2 - CONN_FROM_EDGE - CONN_D / 2 - CONN_CLR
    with BuildPart() as cp:
        # 板
        Box(OUTER, OUTER, COVER_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
        fillet(cp.edges().filter_by(Axis.Z), CORNER_R)
        chamfer(cp.faces().sort_by(Axis.Z)[-1].edges(), FRONT_CHAMFER)
        # リップ (板の下、本体内壁に沿う枠)
        with BuildSketch(Plane.XY):
            RectangleRounded(LIP_OUT, LIP_OUT, 0.8 - LIP_CLR)
            RectangleRounded(LIP_IN, LIP_IN, 0.5, mode=Mode.SUBTRACT)
        extrude(amount=-LIP_H)
        # スナップ突起 (左右、リップ先端寄り。下側 SNAP_PROUD 分は 45° テーパ、上面は垂直の掛かり面)
        z_tip = -LIP_H
        for sx in (-1, 1):
            x0 = sx * LIP_OUT / 2
            pts = [
                (x0 - sx * 0.3, z_tip + 0.2),
                (x0 + sx * SNAP_PROUD, z_tip + 0.2 + SNAP_PROUD),
                (x0 + sx * SNAP_PROUD, z_tip + 0.2 + SNAP_H),
                (x0 - sx * 0.3, z_tip + 0.2 + SNAP_H),
            ]
            with BuildSketch(Plane.XZ):
                Polygon(*pts, align=None)
            extrude(amount=SNAP_LEN / 2, both=True)
        # ケーブル切り欠き (コネクタ直上から上端まで)
        with Locations((CONN_X, (notch_y0 + OUTER / 2 + 1) / 2, 0)):
            Box(notch_w, OUTER / 2 + 1 - notch_y0, 20, mode=Mode.SUBTRACT)
        # 基板押さえの柱 (板の内面 Z=0 から基板裏面まで + 押し込み量)
        if BOARD_PRESS:
            post_len = DEPTH - (BOARD_Z + BOARD_T) + PRESS_INTERF
            corners = [(sx * HOLE_OFF, sy * HOLE_OFF) for sx in (-1, 1) for sy in (-1, 1)]
            with BuildSketch(Plane.XY):
                with Locations(*corners):
                    Circle(PRESS_POST_D / 2)
            extrude(amount=-post_len)
            if PRESS_PIN:
                with BuildSketch(Plane.XY.offset(-post_len)):
                    with Locations(*corners):
                        Circle(PRESS_PIN_D / 2)
                extrude(amount=-PRESS_PIN_LEN)
    return cp.part.moved(Location((0, 0, DEPTH)))


# ---------------------------------------------------------------------------
# 3. マウント mount (housing 座標系の組み立て位置。X = 厚さ、Y = 上、Z = 後ろ)
# ---------------------------------------------------------------------------
def build_mount() -> Part:
    # ピボット中心を原点とするローカル座標 (u = +Y 上, v = +Z 後ろ) で作り、
    # FIXED_TILT_DEG だけ X 軸まわりに回してピボット位置へ移す。D 穴は housing 座標の向きで切る。
    # → 組み立てると本体がマウントに対して FIXED_TILT_DEG 俯角になる。
    v_back = MONITOR_FACE_DZ            # アーム背面 (モニタ前面に当たる)
    v_front = v_back - ARM_W            # アーム前面
    v_end = v_back + PAD_LEN            # パッド後端
    u_top = PAD_TOP_DY                  # パッド上面
    u_pad = MONITOR_TOP_DY              # パッド下面 (モニタ上面に載る)
    u_bot = -ARM_BELOW                  # アーム下端
    with BuildPart() as mp:
        with BuildSketch(Plane.YZ) as sk:   # ローカル x = Y (u), y = Z (v)
            Polygon((u_top, v_front), (u_top, v_end), (u_pad, v_end), (u_pad, v_back),
                    (u_bot, v_back), (u_bot, v_front), align=None)
            Circle(MOUNT_BOSS_R)        # ピボットボス (原点)
            vs = sk.vertices()
            # 角の処理 (スケッチローカル座標: x=u, y=v)
            fillet(vs.filter_by_position(Axis.X, u_top - 0.1, u_top + 0.1)
                     .filter_by_position(Axis.Y, v_front - 0.1, v_front + 0.1), 1.5)   # 曲がり角の外側 (ボス交点までの辺が短いので小さめ)
            fillet(vs.filter_by_position(Axis.Y, v_end - 0.1, v_end + 0.1), 2.0)        # パッド後端
            fillet(vs.filter_by_position(Axis.X, u_bot - 0.1, u_bot + 0.1), 2.0)        # アーム下端
            # ボスとアーム前面の付け根 (凹 R)。上側はパッド角までの辺が短いので小さめ
            for lo, hi, r in ((-MOUNT_BOSS_R, 0.0, 1.0), (0.0, MOUNT_BOSS_R, 0.5)):
                try:
                    conc = (sk.vertices().filter_by_position(Axis.X, lo, hi)
                            .filter_by_position(Axis.Y, v_front - 0.1, v_front + 0.1))
                    fillet(conc, r)
                except Exception as e:
                    print("mount concave fillet skipped:", e)
        extrude(amount=MOUNT_T / 2, both=True)
        try:
            chamfer(mp.edges().filter_by(Plane.YZ), 0.5)
        except Exception as e:
            print("mount chamfer skipped:", e)
        if PLATES:
            xc = PLATE_SIDE * (PLATE_W / 2 - MOUNT_T / 2)   # プレート中心 X (バーの片側面と面一)
            # パッド側プレート: 下面をパッド下面 (モニタ上面に当たる面) と面一に
            with BuildSketch(Plane.XZ.offset(-u_pad)):      # 法線 -Y、原点 Y = u_pad
                with Locations((xc, (PAD_PLATE_V0 + v_end) / 2)):
                    RectangleRounded(PLATE_W, v_end - PAD_PLATE_V0, PLATE_R)
            extrude(amount=-PLATE_T)                         # +Y 側へ
            # アーム側プレート: 背面をアーム背面 (モニタ前面に当たる面) と面一に。
            # 脚より外側 (|X| >= WEB_X0) では上まで伸ばしてパッド側プレートと繋ぎ、連続した L 面にする
            x_out = PLATE_SIDE * (PLATE_W - MOUNT_T / 2)       # プレート外端 X
            with BuildSketch(Plane.XY.offset(v_back)):       # 法線 +Z、原点 Z = v_back
                with Locations((xc, (u_bot + ARM_PLATE_U0) / 2)):
                    RectangleRounded(PLATE_W, ARM_PLATE_U0 - u_bot, PLATE_R)
                with Locations(((PLATE_SIDE * WEB_X0 + x_out) / 2, (ARM_PLATE_U0 + u_pad + PLATE_T) / 2)):
                    Rectangle(abs(x_out - PLATE_SIDE * WEB_X0), u_pad + PLATE_T - ARM_PLATE_U0)
            extrude(amount=-PLATE_T)                         # -Z 側へ
    part = mp.part.rotate(Axis.X, FIXED_TILT_DEG).moved(Location((0, PIVOT_Y, PIVOT_Z)))
    # ピボット D 穴 (面取り後に開けて、穴の縁は面取りしない)。向きは脚の穴と同じ
    span = PLATE_W + MOUNT_T
    with BuildPart() as mp2:
        add(part)
        with BuildSketch(Plane.YZ.offset(-span)):
            d_profile(MOUNT_HOLE_D / 2, HOLE_FLAT_FROM_AXIS, (PIVOT_Y, PIVOT_Z))
        extrude(amount=2 * span, mode=Mode.SUBTRACT)
    return mp2.part


# ---------------------------------------------------------------------------
# 4. ヒンジピン pin (軸 = X 方向、寝かせた印刷向きで作成。底面の平面が D の直線部)
# ---------------------------------------------------------------------------
def build_pin(d: float, marks: int) -> Part:
    with BuildPart() as pp:
        # 軸 (X = 0..PIN_LEN)
        with BuildSketch(Plane.YZ):
            Circle(d / 2)
        extrude(amount=PIN_LEN)
        tip = pp.faces().sort_by(Axis.X)[-1]
        chamfer(tip.edges(), PIN_TIP_CHAMFER)
        # 頭 (X = -PIN_HEAD_T..0)
        with BuildSketch(Plane.YZ):
            Circle(PIN_HEAD_D / 2)
        extrude(amount=-PIN_HEAD_T)
        # 太さの目印: 頭の外面に小さな点を marks 個
        with BuildSketch(Plane.YZ.offset(-PIN_HEAD_T)):
            with GridLocations(1.6, 0, marks, 1):
                Circle(0.4)
        extrude(amount=0.4, mode=Mode.SUBTRACT)
        # D 形の平面 (軸と頭を通しでカット。ベッドに平らに置ける)
        z_cut = -PIN_FLAT_FROM_AXIS
        with Locations((PIN_LEN / 2 - PIN_HEAD_T / 2, 0, z_cut - 5)):
            Box(PIN_LEN + PIN_HEAD_T + 2, PIN_HEAD_D + 2, 10, mode=Mode.SUBTRACT)
    return pp.part.moved(Location((0, 0, PIN_FLAT_FROM_AXIS)))


# ---------------------------------------------------------------------------
# 部品の生成
# ---------------------------------------------------------------------------
housing = build_housing()
cover = build_cover()
mount = build_mount()
pins = [(d, build_pin(d, i + 1)) for i, d in enumerate(PIN_SIZES)]

# 印刷向きの部品 (housing はそのまま、cover は裏返して Z=0 に、mount は寝かせる)
housing_print = housing
cover_print = cover.rotate(Axis.Y, 180).moved(Location((0, 0, DEPTH + COVER_T)))
# mount はバーと面一の側 (プレートが張り出していない側) を下にして寝かせる
mount_print = place(mount.rotate(Axis.Y, -90 * PLATE_SIDE), 0, 0)

if __name__ == "__main__":
    OUT_DIR.mkdir(exist_ok=True)
    export_stl(housing_print, str(OUT_DIR / "housing.stl"))
    export_stl(cover_print, str(OUT_DIR / "cover.stl"))
    export_stl(mount_print, str(OUT_DIR / "mount.stl"))
    export_step(housing_print, str(OUT_DIR / "housing.step"))
    export_step(cover_print, str(OUT_DIR / "cover.step"))
    export_step(mount_print, str(OUT_DIR / "mount.step"))
    for d, pin in pins:
        export_stl(pin, str(OUT_DIR / f"pin_{d:.1f}.stl"))
    # 全部品を並べた 3MF (Bambu Studio にそのまま読み込める)
    pin_row = [place(pin, -45, -45 - 12 * i) for i, (_, pin) in enumerate(pins)]
    m = Mesher()
    m.add_shape([
        place(housing_print, -35, 10),
        place(cover_print, 25, 10),
        place(mount_print, 25, -40),
    ] + pin_row, part_number="webcamera_holder")
    m.write(str(OUT_DIR / "webcamera_holder.3mf"))
    # 再印刷用: 裏蓋 + ピンだけの 3MF
    m = Mesher()
    m.add_shape([place(cover_print, 0, 0)] + [place(pin, -45, -15 + 12 * i)
                                              for i, (_, pin) in enumerate(pins)], part_number="reprint")
    m.write(str(OUT_DIR / "reprint_cover_pins.3mf"))
    print("exported to", OUT_DIR)

    if show is not None and not os.environ.get("NO_SHOW"):
        try:
            # 印刷向きの部品
            show(
                housing_print,
                cover_print.moved(Location((60, 10, 0))),
                mount_print.moved(Location((60, -45, 0))),
                *[pin.moved(Location((100, -45 + 12 * i, 0))) for i, (_, pin) in enumerate(pins)],
                names=["housing", "cover", "mount"] + [f"pin_{d:.1f}" for d, _ in pins],
                colors=["lightblue", "gray", "dimgray"] + ["orange"] * len(pins),
            )
        except Exception as e:  # viewer 未起動など
            print("show() skipped:", e)
