"""Shared industrial robot palette (sRGB); visual properties only."""

PALETTE = {
    'body': ((0.10, 0.12, 0.15), 0.35, 0.48),
    'finger': ((47 / 255, 144 / 255, 206 / 255), 0.10, 0.42),  # #2F90CE
}


def link_finish(name):
    return 'finger' if 'gripper' in name else 'body'



def link_color(name):
    return PALETTE[link_finish(name)][0]


def linear_color(rgb):
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb)


def apply_isaac_materials():
    # Called only after Kit startup and the robot reference has been spawned.
    import isaaclab.sim as sim
    import omni.usd
    from pxr import UsdShade
    stage = omni.usd.get_context().get_stage()
    materials = {}
    for name, (rgb, metallic, roughness) in PALETTE.items():
        path = f'/World/RobotLooks/{name}'
        cfg = sim.PreviewSurfaceCfg(diffuse_color=linear_color(rgb), metallic=metallic, roughness=roughness)
        cfg.func(path, cfg)
        materials[name] = UsdShade.Material(stage.GetPrimAtPath(path))
    root = stage.GetPrimAtPath('/World/F14')
    # Override imported ancestor bindings at each rigid link; mesh instances
    # can remain shared. Strong binding also overrides nested STL materials.
    for prim in root.GetChildren():
        name = prim.GetName()
        if name == 'base_link' or name.endswith('_link'):
            visuals = prim.GetChild("visuals")
            UsdShade.MaterialBindingAPI.Apply(visuals if visuals.IsValid() else prim).Bind(
                materials[link_finish(name)], UsdShade.Tokens.strongerThanDescendants)
