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


def random_rot_objective(phase: int,
                         dominant_frame: str,
                         reference_frame: str,
                         soft_rot_slack: list[float]=[.01, .4]) -> str:
    
    rot_type = np.random.randint(0, 3)
    sign = np.random.choice([-1, 1])
    dir = np.random.choice(['X', 'Y', 'Z'])

    if rot_type == 0: # Hard rotation constraint
        contraint = f"komo.addObjective([{phase}], ry.FS.vector{dir}, ['{dominant_frame}'], ry.OT.eq, [1e1], [0., 0., {sign}])\n"

    elif rot_type == 1:
        contraint = f"komo.addObjective([{phase}], ry.FS.vector{dir}Diff, ['{dominant_frame}', '{reference_frame}'], ry.OT.eq, [1e1])\n"
    
    else: # Soft rotation constraints
        slack = np.round(np.random.uniform(*soft_rot_slack), 2)
        contraint = f"komo.addObjective([{phase}], ry.FS.scalarProduct{dir}Z, ['{reference_frame}', '{dominant_frame}'], ry.OT.ineq, [{-sign}e1], [{sign*(1-slack)}])\n"
    
    return contraint


def random_pick(times: list[float],
                end_effector: str,
                target_obj: str,
                hard_grasp_prob: float=.2,
                specific_orientation_prob: float=.5,
                pick_depth_max: float=.05) -> str:
    
    if np.random.random() < hard_grasp_prob:
        komo_text = f"komo.addObjective({times}, ry.FS.positionDiff, ['{end_effector}', '{target_obj}'], ry.OT.eq, [1e1])\n"
    else:
        depth = np.round(np.random.uniform(0, pick_depth_max), 3)
        komo_text = f"komo.addObjective({times}, ry.FS.negDistance, ['{end_effector}', '{target_obj}'], ry.OT.eq, [1e1], [{depth}])\n"

    if np.random.random() < specific_orientation_prob:
        komo_text += random_rot_objective(times[0], end_effector, target_obj)
    
    return komo_text


def generate_random_pick(end_effector_frames: list[str],
                         movable_object_frames: list[str],
                         world_attach_frames: list[str],
                         all_frames: list[str],
                         place_prob: float=.9,
                         approach_prob: float=.5,
                         specific_orientation_prob: float=.5,
                         retract_prob: float=.5,
                         slices: int=8,
                         hard_grasp_prob: float=.2,
                         drop_prob: float=.2,
                         drop_height: float=.7,
                         specific_place_prob: float=.1,
                         place_rot_prob: float=.5,
                         accumulated_collisions: bool=True) -> str:
    
    end_effector = np.random.choice(end_effector_frames)
    movable_object = np.random.choice(movable_object_frames)
    world_frame = np.random.choice(world_attach_frames)
    approach = np.random.random() < approach_prob
    place = np.random.random() < place_prob
    drop = np.random.random() < drop_prob
    retract = place and (not drop) and np.random.random() < retract_prob

    # TODO: if two endeffectors, switch picked from one endeff to another

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
    final_komo += random_pick([1], end_effector, movable_object, hard_grasp_prob, specific_orientation_prob)
        
    ### PLACE ###
    if place:
        final_komo += f"komo.addModeSwitch([1, 2], ry.SY.stable, ['{end_effector}', '{movable_object}'], True)\n"
        
        if drop:
            final_komo += f"komo.addObjective([2], ry.FS.position, ['{movable_object}'], ry.OT.eq, [0, 0, 1e1], [0, 0, {drop_height}])\n"
        
        else:
            if np.random.random() < specific_place_prob:
                target = np.round(np.random.uniform(-2, 2, 3), 2).tolist()
                target[2] = float(np.round(np.random.uniform(.5, 1.5), 2))
                final_komo += f"komo.addObjective([2], ry.FS.position, ['{movable_object}'], ry.OT.eq, [1e1], {target})\n"
            
            else: # Place on surface
                surfaces = all_frames.copy()
                # surfaces.remove(movable_object)
                # surfaces.remove(end_effector)
                for f in all_frames:
                    if f.startswith("r_"):
                        surfaces.remove(f)
                for f in movable_object_frames:
                    surfaces.remove(f)

                surface = np.random.choice(surfaces)
                z_dist = np.round(np.random.uniform(0., .1), 3)
                xy_slack = np.round(np.random.uniform(0., .3), 3)
                
                # On top of surface
                # TODO: add alternative with positionDiff
                final_komo += f"komo.addObjective([2], ry.FS.positionRel, ['{surface}', '{movable_object}'], ry.OT.ineq, [0, 0, -1e1])\n"
                # if np.random.choice([0, 1]):
                final_komo += f"komo.addObjective([2], ry.FS.negDistance, ['{surface}', '{movable_object}'], ry.OT.ineq, [-1e1], [{-z_dist}])\n"
                final_komo += f"komo.addObjective([2], ry.FS.negDistance, ['{surface}', '{movable_object}'], ry.OT.ineq, [1e1], [0])\n"
                # else:
                #     feature_type = np.random.choice(["positionDiff", "positionRel"])
                #     final_komo += f"komo.addObjective([2], ry.FS.{feature_type}, ['{surface}', '{movable_object}'], ry.OT.ineq, [0, 0, 1e1], [0, 0, {z_dist + .1}])\n"
                
                # Relative xy of place
                # TODO: alternatives for relative xy, measure surface dimensions?
                feature_type = np.random.choice(["positionDiff", "positionRel"])
                final_komo += f"komo.addObjective([2], ry.FS.{feature_type}, ['{movable_object}', '{surface}'], ry.OT.ineq, [1e1, 1e1, 0], [{xy_slack}, {xy_slack}, 0])\n"
                final_komo += f"komo.addObjective([2], ry.FS.{feature_type}, ['{movable_object}', '{surface}'], ry.OT.ineq, [-1e1, -1e1, 0], [{-xy_slack}, {-xy_slack}, 0])\n"

            if np.random.random() < place_rot_prob:
                final_komo += random_rot_objective(2, movable_object, surface)
                

    ### RETRACT ###
    if retract:
        final_komo += f"komo.addModeSwitch([2, 3], ry.SY.stable, ['{world_frame}', '{movable_object}'], False)\n"

    return final_komo


def generate_random_pivot(end_effector_frames: list[str],
                          jointed_frames: list[str],
                          joint_frames: list[str],
                          slices: int=8,
                          max_back_and_forth_phases: int=8,
                          accumulated_collisions: bool=True) -> str:
    
    # Frames setup
    end_effector = np.random.choice(end_effector_frames)
    man_idx = np.random.randint(0, len(joint_frames))
    jointed_frame = jointed_frames[man_idx]
    joint_frame = joint_frames[man_idx]
    back_and_forth_phases = np.random.randint(1, max_back_and_forth_phases+1)
    
    # Write problem
    phases = back_and_forth_phases+1
    final_komo = init_komo(phases, slices, accumulated_collisions)

    final_komo += f"komo.addObjective([0, 1], ry.FS.pose, ['{jointed_frame}'], ry.OT.eq, [1e1], [], 1)\n"
    final_komo += f"komo.addObjective([0, 1], ry.FS.qItself, ['{joint_frame}'], ry.OT.eq, [1e1], [], 1)\n"
    final_komo += f"komo.addObjective([], ry.FS.position, ['omnibase'], ry.OT.sos, [1e0], [], 1)\n"
    # final_komo += f"komo.addObjective([0, 2], ry.FS.pose, ['sinks'], ry.OT.eq, [1e1], [])\n"

    qTarget = np.round(np.random.uniform(-2, 2), 3)
    signs = [-1, 1]
    for i in range(back_and_forth_phases):
        final_komo += f"komo.addObjective([{i+2}], ry.FS.qItself, ['{joint_frame}'], ry.OT.eq, [1e1], [{qTarget*signs[i%2]}])\n"

    final_komo += random_pick([1, phases], end_effector, jointed_frame, specific_orientation_prob=0, pick_depth_max=.1)

    if np.random.random() < 1.:
        final_komo += f"komo.addObjective([1, {phases}], ry.FS.angularVel, ['{end_effector}'], ry.OT.eq, [1e1])\n"
    else:
        final_komo += f"komo.addObjective([1, {phases}], ry.FS.vectorX, ['{end_effector}'], ry.OT.eq, [1e1], [], 1)\n"
        final_komo += f"komo.addObjective([1, {phases}], ry.FS.vectorY, ['{end_effector}'], ry.OT.eq, [1e1], [], 1)\n"
        final_komo += f"komo.addObjective([1, {phases}], ry.FS.vectorZ, ['{end_effector}'], ry.OT.eq, [1e1], [], 1)\n"

    # Relative finger during push:
    # Stay at POA
    # final_komo += f"komo.addModeSwitch([1, 2], ry.SY.stable, ['{end_effector}', '{jointed_frame}'], True)\n"
    # final_komo += f"komo.addObjective([1, 2], ry.FS.positionRel, ['{finger}', '{poa}'], ry.OT.eq, [1e1])"

    return final_komo


# For mobile base
def generate_random_base_move(slices: int=8,
                              accumulated_collisions: bool=True) -> str:
    
    final_komo = init_komo(2, slices, accumulated_collisions)
    return final_komo
