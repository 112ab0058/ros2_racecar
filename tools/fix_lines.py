#!/usr/bin/env python3
import math, re

SDF_PATH = '/home/ray/ros2_ws/src/wro2026_sim/worlds/wro2026_field.sdf'
LINE_W = 0.02
LINE_H = 0.001
Z      = 0.006

OR = ("1.0", "0.4", "0")
BL = ("0", "0.2", "1.0")

OUTER = 1.59   # 外牆內緣
INNER = 0.59   # 內牆外緣
D = 0.92       # 距直道入口 420mm

def make_model(name, ax, ay, bx, by, rgb):
    cx = (ax+bx)/2
    cy = (ay+by)/2
    length = math.sqrt((ax-bx)**2 + (ay-by)**2)
    angle = math.atan2(by-ay, bx-ax)
    r, g, b = rgb
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {Z} 0 0 {angle:.4f}</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>{length:.4f} {LINE_W} {LINE_H}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

# 內牆角落在 ±0.59（外緣），線端點接到角落
# 外牆端點：在外牆內緣，距直道入口 D=0.92

new_models = [
    # NW 左上：橘=左外牆→左上內牆角，藍=上外牆→左上內牆角
    make_model("orange_line_nw", -OUTER,  D,    -INNER, INNER, OR),
    make_model("blue_line_nw",   -D,      OUTER,-INNER, INNER, BL),

    # NE 右上：橘=上外牆→右上內牆角，藍=右外牆→右上內牆角
    make_model("orange_line_ne",  D,      OUTER, INNER, INNER, OR),
    make_model("blue_line_ne",    OUTER,  D,     INNER, INNER, BL),

    # SE 右下：橘=右外牆→右下內牆角，藍=下外牆→右下內牆角
    make_model("orange_line_se",  OUTER, -D,     INNER,-INNER, OR),
    make_model("blue_line_se",    D,     -OUTER, INNER,-INNER, BL),

    # SW 左下：橘=下外牆→左下內牆角，藍=左外牆→左下內牆角
    make_model("orange_line_sw", -D,     -OUTER,-INNER,-INNER, OR),
    make_model("blue_line_sw",   -OUTER, -D,    -INNER,-INNER, BL),
]

with open(SDF_PATH, 'r') as f:
    content = f.read()

content = re.sub(
    r'\s*<model name="(?:orange|blue)_line_[^"]*">.*?</model>',
    '', content, flags=re.DOTALL
)

new_block = '\n'.join(new_models)
content = content.replace('</world>', new_block + '\n  </world>')

with open(SDF_PATH, 'w') as f:
    f.write(content)

print("✅ 完成，端點對到內牆角落")