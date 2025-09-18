import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from cmap import Colormap
import colorspacious as cs


def generate_colormap_lut_cpp(
    cmap_name: str, num_colors: int = 1024, cmap_name_override: str | None = None
):
    cmap_name_out = cmap_name if cmap_name_override is None else cmap_name_override
    filename = f"src/cmaps/{cmap_name_out}.h"
    array_name = f"{cmap_name_out}_colormap"

    # Get the colormap
    cmap = Colormap(cmap_name).to_mpl()

    ts = np.linspace(0, 1, num_colors)
    lut = cmap(ts)[:, :3]  # Omit alpha channel

    lut_255 = (lut * 255).astype(int)

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


if __name__ == "__main__":
    # generate_colormap_lut_cpp("guppy_r", cmap_name_override="guppy_r_hq")
    # generate_colormap_lut_cpp("cosmic", cmap_name_override="cosmic_hq")
    # generate_colormap_lut_cpp("pride", cmap_name_override="pride_hq")
    # generate_colormap_lut_cpp("sunset", cmap_name_override="sunset_hq")
    generate_colormap_lut_cpp("gem_r", cmap_name_override="gem_r_hq")
