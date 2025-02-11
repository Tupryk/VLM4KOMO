import numpy as np
from lookup import contraint_types
from utils import get_rand_scale_vec


def init_komo(phases_count: int,
              slices: int,
              accumulated_collisions: bool)-> str:
    final_komo = ""
    final_komo += f"komo = ry.KOMO(C, {phases_count}, {slices}, 2, {accumulated_collisions})\n"
    final_komo += f"komo.addControlObjective([], 0, 1e-2)\n"
    final_komo += f"komo.addControlObjective([], 1, 1e-1)\n"
    final_komo += f"komo.addControlObjective([], 2, 1e0)\n"
    final_komo += f"komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)\n"
    if accumulated_collisions:
        final_komo += f"komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)\n"
    return final_komo


def gen_rand_contraint(frame_names: list[str],
                       lucky_frames: list[str],
                       features: list[str],
                       lucky_features: list[str],
                       start: float,
                       end: float = None,
                       prob_lucky_frame: float=.7,
                       target_prob: float=.7,
                       prob_lucky_feature: float=.3,
                       scale_as_vec_prob: float=.2) -> str:

    if end != None:
        times = f"[{start}, {end}]"
    else:
        times = f"[{start}]"

    if prob_lucky_feature:
        feature_name = np.random.choice(lucky_features)
    else:
        feature_name = np.random.choice(list(features.keys()))

    contraint_type = np.random.choice(contraint_types)
    # scale_pow = np.random.randint(-2, 3) # As matrix?
    if feature_name == "negDistance":
        scale_pow = 1
    else:
        scale_pow = np.random.choice([-1, 1])  # As matrix?

    scale_as_vec = np.random.random() < scale_as_vec_prob
    if features[feature_name][1] != 1 and scale_as_vec:
        scale_vec = get_rand_scale_vec(features[feature_name][1])
        scale = "["
        for v in scale_vec:
            if v == 1:
                scale += f"1e{scale_pow}, "
            else:
                scale += "0, "
        scale += "]"
        scale = scale.replace(", ]", "]")
    else:
        scale = f"[1e{scale_pow}]"

    frames_count = features[feature_name][0]
    frames = np.random.choice(
        frame_names, frames_count, replace=False).tolist()

    if np.random.random() < prob_lucky_frame:
        lucky_frame = np.random.choice(lucky_frames)
        if frames_count == 2 and np.random.choice([0, 1]):
            frames[1] = str(lucky_frame)
        else:
            frames[0] = str(lucky_frame)

    specific_target = np.random.random() < target_prob
    if specific_target:
        if feature_name == "negDistance":
            target = np.round(np.random.uniform(-2, 0, 1), 3).tolist()
        else:
            if feature_name == "qItself":
                target_range = np.pi*2
            else:
                target_range = 2
            target = np.round(np.random.uniform(-target_range,
                              target_range, (features[feature_name][1])), 3).tolist()
        contraint_line = f"komo.addObjective({times}, ry.FS.{feature_name}, {frames}, ry.OT.{contraint_type}, {scale}, {target})\n"
    else:
        contraint_line = f"komo.addObjective({times}, ry.FS.{feature_name}, {frames}, ry.OT.{contraint_type}, {scale})\n"

    return contraint_line


def random_start_end(phases_count: int, uniform_prob: float) -> tuple[float, float]:
    if np.random.random() < uniform_prob:
        start = np.random.uniform(0, phases_count-.1)
        start = np.round(start, 1)
        end = np.random.uniform(start+.1, phases_count)
        end = np.round(end, 1)
    else:
        start = np.random.randint(0, phases_count)
        end = np.random.randint(start+1, phases_count+1)

    return start, end


def generate_random_komo(frame_names: list[str],
                         features: dict,
                         lucky_frames: list[str],
                         lucky_features: list[str],
                         prob_lucky_frame: float=.7,
                         prob_lucky_feature: float=.3,
                         max_phases: int=4,
                         max_phase_contraints: int=4,
                         prob_sub: float=.5,
                         max_sub: int=4,
                         uniform_sub_prob: float=.2,
                         prob_mode_switch: float=.5,
                         max_mode_switches: int=2,
                         uniform_mode_switch_prob: float=.1,
                         slices: int=8,
                         target_prob: float=.7,
                         scale_as_vec_prob: float=.2,
                         accumulated_collisions: bool=True) -> str:
    
    phases_count = np.random.randint(1, max_phases+1)
    sub_count = np.random.randint(0, max_sub+1)
    mode_switch_count = np.random.randint(0, max_mode_switches+1)

    final_komo = init_komo(phases_count, slices, accumulated_collisions)

    # Main phases
    for i in range(phases_count):
        repeats = np.random.randint(1, max_phase_contraints+1)
        for _ in range(repeats):
            final_komo += gen_rand_contraint(frame_names,
                                             lucky_frames,
                                             features,
                                             lucky_features,
                                             i+1,
                                             None,
                                             prob_lucky_frame,
                                             target_prob,
                                             prob_lucky_feature,
                                             scale_as_vec_prob)

    # Sub-motions
    if np.random.random() < prob_sub:
        for i in range(sub_count):
            start, end = random_start_end(phases_count, uniform_sub_prob)
            final_komo += gen_rand_contraint(frame_names,
                                             lucky_frames,
                                             features,
                                             lucky_features,
                                             start,
                                             end,
                                             prob_lucky_frame,
                                             target_prob,
                                             prob_lucky_feature,
                                             scale_as_vec_prob)

    # Mode Switches
    if phases_count >= 2 and np.random.random() < prob_mode_switch:
        for i in range(mode_switch_count):
            first_switch = i == 0
            start, end = random_start_end(
                phases_count, uniform_mode_switch_prob)
            frames = np.random.choice(frame_names, 2, replace=False).tolist()
            if np.random.random() < prob_lucky_frame:
                lucky_frame = np.random.choice(lucky_frames)
                frame_idx = np.random.choice([0, 1])
                frames[frame_idx] = lucky_frame
            final_komo += f"komo.addModeSwitch([{start}, {end}], ry.SY.stable, {frames}, {first_switch})\n"

    return final_komo


def generate_random_pick(end_effector_frames: list[str],
                         movable_object_frames: list[str],
                         world_attach_frames: list[str],
                         place_prob: float=.9,
                         approach_prob: float=.5,
                         specific_orientation_prob: float=.5,
                         retract_prob: float=.5,
                         slices: int=8,
                         hard_grasp_prob: float=.2,
                         drop_prob: float=.2,
                         drop_height: float=.7,
                         specific_place_prob: float=.2,
                         accumulated_collisions: bool=True) -> str:
    
    end_effector = np.random.choice(end_effector_frames)
    movable_object = np.random.choice(movable_object_frames)
    world_frame = np.random.choice(world_attach_frames)
    approach = np.random.random() < approach_prob
    place = np.random.random() < place_prob
    drop = np.random.random() < drop_prob
    retract = place and (not drop) and np.random.random() < retract_prob

    phases_count = 1
    if place:
        phases_count += 1
    if retract:
        phases_count += 1

    final_komo = init_komo(phases_count, slices, accumulated_collisions)

    ### APPROACH ###
    if approach:
        pass
    
    ### PICK ###
    if np.random.random() < hard_grasp_prob:
        final_komo += f"komo.addObjective([1], ry.FS.positionDiff, ['{end_effector}', '{movable_object}'], ry.OT.eq, [1e1])\n"
    else:
        depth = np.round(np.random.uniform(0, .05), 3)
        final_komo += f"komo.addObjective([1], ry.FS.negDistance, ['{end_effector}', '{movable_object}'], ry.OT.eq, [1e1], [{depth}])\n"

    ### PLACE ###
    if place:
        final_komo += f"komo.addModeSwitch([1, 2], ry.SY.stable, ['{end_effector}', '{movable_object}'], True)\n"
        if drop:
            final_komo += f"komo.addObjective([2], ry.FS.position, ['{movable_object}'], ry.OT.eq, [0, 0, 1e1], [0, 0, {drop_height}])\n"
        elif np.random.random() < specific_place_prob:
            target = np.round(np.random.uniform(-2, 2, 3), 2).tolist()
            target[2] = float(np.round(np.random.uniform(.5, 1.5), 2))
            final_komo += f"komo.addObjective([2], ry.FS.position, ['{movable_object}'], ry.OT.eq, [1e1], {target})\n"
        else:
            pass

    ### RETRACT ###
    if retract:
        final_komo += f"komo.addModeSwitch([2, 3], ry.SY.stable, ['{world_frame}', '{movable_object}'], False)\n"

    return final_komo
