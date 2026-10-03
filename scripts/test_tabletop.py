"""Bounded real GPU test: random cube support, open box floor/walls, RGB.

Run with env_isaaclab: python scripts/test_tabletop.py --headless --device cuda:0
Snapshots/results default to outputs/tabletop_check. No teleop sockets used.
"""
import argparse
import json
from pathlib import Path
import sys
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def run(app, args):
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from config import tabletop_config as cfg
    from simulation.f14_scene import create_scene, resolve_joint_ids, initialize_home, semantic_state
    from simulation.cameras import create_cameras, raw_observation, display_rgb_copies, save_snapshots
    from simulation.quest_view_compositor import compose_quest_view
    from simulation.tabletop import sample_cube_poses, table_surface_height

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 60, device=args.device))
    cameras = {}
    try:
        robot = create_scene(scene_seed=args.scene_seed)
        cubes = RigidObject(RigidObjectCfg(prim_path="/World/Cubes/Cube_.*"))
        cameras = create_cameras(debug=True)
        sim.reset()
        for camera in cameras.values():
            camera.update(0., force_recompute=True)
        arm_ids, finger_ids = resolve_joint_ids(robot)
        initialize_home(robot, arm_ids, finger_ids)
        expected = sample_cube_poses(args.scene_seed)
        surface = table_surface_height()
        result = {"seed": args.scene_seed, "spawn_positions": [p for p, _ in expected]}
        for step in range(240):
            if not app.is_running():
                raise RuntimeError("Tabletop check interrupted")
            if step in (100, 180):
                # Reuse the real task cube: drop into the open box, then push
                # it toward the left wall to prove the container is collidable.
                state = cubes.data.root_state_w[:1].clone()
                state[0, :3] = torch.tensor(
                    (cfg.BOX_CENTER_XY[0], cfg.BOX_CENTER_XY[1],
                     surface + cfg.BOX_SIZE[2] + cfg.CUBE_SIZE), device=sim.device)
                state[0, 3:7] = torch.tensor((1., 0., 0., 0.), device=sim.device)
                state[0, 7:] = 0.
                if step == 180:
                    state[0, 1] += .04
                    state[0, 2] = surface + cfg.BOX_BOTTOM_THICKNESS + cfg.CUBE_SIZE / 2 + .001
                    state[0, 8] = .8
                cubes.write_root_state_to_sim(state, env_ids=torch.tensor([0], device=sim.device))
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(1 / 60)
            cubes.update(1 / 60)
            for camera in cameras.values():
                camera.update(1 / 60)
            if step % 2 == 1:
                sim.render()
            if step in (89, 169, 239):
                positions = cubes.data.root_pos_w.cpu().numpy()
                speeds = torch.linalg.norm(cubes.data.root_lin_vel_w, dim=1).cpu().numpy()
                if step == 89:
                    np.testing.assert_allclose(positions[:, :2], np.array([p[:2] for p, _ in expected]), atol=.003)
                    np.testing.assert_allclose(positions[:, 2], surface + cfg.CUBE_SIZE / 2, atol=.002)
                    assert np.max(speeds) < .03, speeds
                    np.testing.assert_allclose(cubes.root_physx_view.get_masses().cpu().numpy(), cfg.CUBE_MASS, atol=1e-6)
                    result["settled_table_positions"] = positions.tolist()
                    suffix = ""
                else:
                    center = positions[0]
                    floor_z = surface + cfg.BOX_BOTTOM_THICKNESS + cfg.CUBE_SIZE / 2
                    assert abs(center[2] - floor_z) < .003, center
                    interior_half = np.array(cfg.BOX_SIZE[:2]) / 2 - cfg.BOX_WALL_THICKNESS
                    assert np.all(np.abs(center[:2] - cfg.BOX_CENTER_XY) < interior_half), center
                    key = "box_floor_position" if step == 169 else "box_wall_stop_position"
                    result[key] = center.tolist()
                    suffix = "_in_box" if step == 169 else "_wall_check"
                observation = raw_observation(cameras, semantic_state(robot, arm_ids, finger_ids))
                images = display_rgb_copies(observation)
                for name, rgb in images.items():
                    assert np.std(rgb.astype(float)) > 1., name
                save_snapshots(args.output_dir, images, compose_quest_view(images, "TABLETOP CHECK"), suffix)
                print(f"Tabletop check step {step + 1}: {positions.tolist()}", flush=True)
        result["passed"] = True
        (args.output_dir / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
        print("TABLETOP CHECK PASSED", json.dumps(result, indent=2), flush=True)
    finally:
        cameras.clear()
        camera = None
        import gc
        gc.collect()
        sim.clear_all_callbacks()
        sim.clear_instance()
        sim.stop()


def main():
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-seed", type=int, default=42)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/tabletop_check")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.enable_cameras = True
    args.output_dir.mkdir(parents=True, exist_ok=True)
    launcher = AppLauncher(args)
    try:
        run(launcher.app, args)
    except Exception:
        traceback.print_exc()
        raise
    finally:
        launcher.app.close()


if __name__ == "__main__":
    main()
