import numpy as np
import matplotlib.pyplot as plt
from cmap import Colormap
import colorspacious as cs


cmap_src_name = "sunset"
cmap_name = "sunset_dark"

filename = f"src/cmaps/{cmap_name}.h"
array_name = f"{cmap_name}_colormap"

num_colors = 256

cmap = Colormap(cmap_src_name).to_mpl()
ts = np.linspace(0, 1, num_colors)
lut = cmap(ts)[:, :3]

# Convert RGB to CAM02-UCS
cam02 = cs.cspace_convert(lut, "sRGB1", "CAM02-UCS")  # J′, a′, b′

# Lightness remap
J_start = cam02[0, 0]  # 35
J_end = cam02[-1, 0]  # 39
J_mid = 60.0
half = num_colors // 2
cam02[:half, 0] = np.linspace(J_start, J_mid, half)
cam02[half:, 0] = np.linspace(J_mid, J_end, num_colors - half)

# Convert back to RGB
lut_adjusted = np.clip(cs.cspace_convert(cam02, "CAM02-UCS", "sRGB1"), 0, 1)
lut_255 = (lut_adjusted * 255).astype(int)


with open(filename, "w") as f:
    f.write(f"// Generated {cmap_name} colormap lookup table\n\n")
    f.write("#pragma once\n\n")
    f.write("#include <cstdint>\n")
    f.write("#include <array>\n")
    f.write("\n")
    f.write(
        f"constexpr std::array<std::array<uint8_t, 3>, {num_colors}> {array_name} = {{{{\n"
    )

    for i in range(num_colors):
        rgb = lut_255[i]
        f.write(f"    {{{rgb[0]}, {rgb[1]}, {rgb[2]}}}")
        if (i + 1) < num_colors:
            f.write(",")
        f.write("\n")

    f.write("}};\n")

print(f"Lookup table written to '{filename}'")
