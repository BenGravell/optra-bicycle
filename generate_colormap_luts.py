import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def generate_colormap_lut_cpp(cmap_name: str, source: str, num_colors: int = 256):
    filename = f"src/cmaps/{cmap_name}.h"
    array_name = f"{cmap_name}_colormap"

    # Get the colormap
    if source == "matplotlib":
        cmap = plt.get_cmap(cmap_name, num_colors)
    elif source == "seaborn":
        cmap = sns.color_palette(cmap_name, as_cmap=True)
    else:
        raise ValueError
    
    ts = np.linspace(0, 1, num_colors)
    lut = cmap(ts)[:, :3]  # Omit alpha channel
    lut_255 = (lut * 255).astype(int)

    with open(filename, "w") as f:
        f.write(f"// Generated {cmap_name} colormap lookup table\n")
        f.write("#include <cstdint>\n")
        f.write("#include <array>\n")
        f.write("\n")
        f.write(f"constexpr std::array<std::array<uint8_t, 3>, {num_colors}> {array_name} = {{{{\n")

        for i in range(num_colors):
            rgb = lut_255[i]
            f.write(f"    {{{rgb[0]}, {rgb[1]}, {rgb[2]}}}")
            if (i+1) < num_colors:
                f.write(",")
            f.write("\n")

        f.write("}};\n")

    print(f"Lookup table written to '{filename}'")

if __name__ == "__main__":
    generate_colormap_lut_cpp("turbo", source="matplotlib")
    generate_colormap_lut_cpp("magma", source="matplotlib")
    generate_colormap_lut_cpp("inferno", source="matplotlib")
    generate_colormap_lut_cpp("viridis", source="matplotlib")

    generate_colormap_lut_cpp("mako", source="seaborn")
    generate_colormap_lut_cpp("rocket", source="seaborn")
    generate_colormap_lut_cpp("flare_r", source="seaborn")
    generate_colormap_lut_cpp("crest_r", source="seaborn")