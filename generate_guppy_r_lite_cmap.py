import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from cmap import Colormap
import colorspacious as cs


def remap_lightness_cam02ucs(rgb_array):
    n = rgb_array.shape[0]

    # Convert from sRGB to CAM02-UCS
    cam02ucs = cs.cspace_convert(rgb_array, "sRGB1", "CAM02-UCS")

    # Create new J' ramp: piecewise linear
    half = n // 2
    ramp = np.zeros(n)
    J1 = 0.8
    J2 = 0.4
    ramp[:half+1] = np.linspace(J1, J2, half+1)
    ramp[half:] = np.linspace(J2, J1, n - half)

    # Scale ramp from [0,1] to same J' scale (0-100 typically)
    new_Jp = ramp * 100.0

    # Replace J' with ramp values
    cam02ucs[:, 0] = new_Jp

    # Convert back to sRGB
    rgb_out = cs.cspace_convert(cam02ucs, "CAM02-UCS", "sRGB1")

    # Clip to valid [0,1] range
    return np.clip(rgb_out, 0, 1)


def generate_colormap_lut_cpp(
    cmap_name: str, num_colors: int = 256, cmap_name_override: str | None = None
):
    cmap_name_out = cmap_name if cmap_name_override is None else cmap_name_override
    filename = f"src/cmaps/{cmap_name_out}.h"
    array_name = f"{cmap_name_out}_colormap"

    # Get the colormap
    cmap = Colormap(cmap_name).to_mpl()

    ts = np.linspace(0, 1, num_colors)
    lut = cmap(ts)[:, :3]  # Omit alpha channel

    fig, ax = plt.subplots(figsize=(8, 2), constrained_layout=True)
    ax.imshow(lut[np.newaxis, :, :], aspect="auto")
    ax.set_title("Original colormap")
    ax.axis("off")
    fig.savefig("preview_old.png")

    lut = remap_lightness_cam02ucs(lut)


    fig, ax = plt.subplots(figsize=(8, 2), constrained_layout=True)
    ax.imshow(lut[np.newaxis, :, :], aspect="auto")
    ax.set_title("Lightness remapped colormap")
    ax.axis("off")
    fig.savefig("preview_new.png")

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
    generate_colormap_lut_cpp("guppy_r", cmap_name_override="guppy_r_lite", num_colors=1024)
