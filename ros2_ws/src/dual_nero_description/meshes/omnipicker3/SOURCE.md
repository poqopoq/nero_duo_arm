# OmniPicker 3 mesh source

Source: AGILINK OmniPicker 3 technical documents, downloaded 2026-11-25.

- Product family: https://www.agilink-ai.com/omni-picker3tdf.html
- Technical documents: https://www.agilink-ai.com/omni-picker3tdf/24.html
- Official URDF package:
  https://cn.agilink-ai.com/uploads/upload/files/20261020/199e5afa56cc02664c89fee7af84db5b.zip
- Official 3D model:
  https://cn.agilink-ai.com/uploads/upload/files/20261020/f410686c9f05a0ab9a9b3801787af90a.zip

The archive package is named `OP3_P0_URDF_EC`, version 1.0.0. Its
`package.xml` declares BSD. The unmodified vendor URDF is retained as
`urdf/OP3_P0_URDF_EC.vendor.urdf`; the project macro only prefixes names,
rewrites package mesh paths, and converts the exported slider limit from
`0..20` to SI metres (`0..0.02`).

The official `meshes/base_link_component_STL/*.STL` files use millimetres and
retain the vendor `scale="0.001 0.001 0.001"`. Top-level `link*.STL` files are
already exported in metres.
