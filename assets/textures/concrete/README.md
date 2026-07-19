# Concrete pavement material

The public build uses Poly Haven's
[Concrete Pavement](https://polyhaven.com/a/concrete_pavement) material by
Charlotte Baglioni, released under
[CC0 1.0 Universal](https://creativecommons.org/publicdomain/zero/1.0/).

Included 2K JPEG channels:

- `concrete_pavement_diff_2k.jpg` -> `VK_FORMAT_R8G8B8A8_SRGB`
- `concrete_pavement_nor_gl_2k.jpg` -> `VK_FORMAT_R8G8B8A8_UNORM`
- `concrete_pavement_ao_2k.jpg` and `concrete_pavement_rough_2k.jpg` -> packed
  at runtime as `VK_FORMAT_R8G8_UNORM`

Metallic remains a constant `0.0` for concrete. The source material covers
approximately 1.8 metres; the renderer currently uses its established
two-metre world tiling.

The previous user-supplied Megascans maps remain usable in local experiments
but are ignored by Git and are not distributed by this repository.
