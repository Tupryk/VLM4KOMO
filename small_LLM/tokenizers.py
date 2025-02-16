import re
import ast
from lookup import features, contraint_types


max_times = 10

def tokenize_komo(text: str, frame_names: list[str]) -> list:
    
    tokens = []
    feature_list = list(features.keys())

    uniform_token = max_times + len(feature_list) + len(contraint_types) + 3
    order_token = uniform_token + 1
    order_token_acc = order_token + 1
    objective_start_token = order_token_acc + 1
    mode_switch_token = order_token_acc + 2
    komo_end_token = order_token_acc + 3

    for line in text.splitlines():
        if "komo.addObjective(" in line:

            if not ("accumulatedCollisions" in line or "jointLimits" in line):
                tokens.append(objective_start_token)

                lists = re.findall(r'\[.*?\]', line)
                params_in_objective = re.split(r',\s*(?![^\[\]]*\])', line)

                ### TIMES ###
                times = lists[0].replace("[", "").replace("]", "").split(", ")
                if not '.' in times[0]:
                    if len(times) > 1:
                        start = int(times[0])
                        end = int(times[1])
                        assert start < 10 and end < 10 and start < end
                        tokens.append(start)
                        tokens.append(end)
                    else:
                        tokens.append(int(times[0]))
                else:
                    tokens.append(uniform_token)
                    if len(times) > 1:
                        tokens.append(uniform_token)

                current_offset = max_times

                ### FEATURE ###
                feature = params_in_objective[1].replace(" ", "").replace("ry.FS.", "")
                feature_idx = feature_list.index(feature) + current_offset
                tokens.append(feature_idx)

                current_offset += len(feature_list)

                ### FRAMES ###
                frames = ast.literal_eval(lists[1])
                for f in frames:
                    frame_idx = frame_names.index(f) + komo_end_token + 1
                    tokens.append(frame_idx)

                ### TYPE ###
                type_ = params_in_objective[3].replace(" ", "").replace(")", "").replace("ry.OT.", "")
                type_idx = contraint_types.index(type_) + current_offset
                tokens.append(type_idx)

                current_offset += len(contraint_types)

                ### FEATURE ###
                scale = ast.literal_eval(lists[2].replace(" ", "").replace(")", ""))
                scale_idx_neg = current_offset
                scale_idx_pos = scale_idx_neg + 1
                scale_idx_zero = scale_idx_neg + 2

                for s in scale:
                    if s > 0:
                        tokens.append(scale_idx_pos)
                    elif s < 0:
                        tokens.append(scale_idx_neg)
                    else:
                        tokens.append(scale_idx_zero)

                current_offset += 3

                ### TARGET ###
                if len(lists) > 3:
                    target = ast.literal_eval(params_in_objective[5].replace(" ", "").replace(")", ""))
                    for _ in target:
                        tokens.append(uniform_token)

                ### ORDER ###
                if line[-2] != "]":
                    if "1" in params_in_objective[-1]:
                        tokens.append(order_token)
                    elif "2" in params_in_objective[-1]:
                        tokens.append(order_token_acc)

        elif "komo.addModeSwitch(" in line:
            tokens.append(mode_switch_token)

            lists = re.findall(r'\[.*?\]', line)
            params_in_objective = re.split(r',\s*(?![^\[\]]*\])', line)

            times = lists[0].replace("[", "").replace("]", "").split(", ")
            if not '.' in times[0]:
                start = int(times[0])
                end = int(times[1])
                assert start < 10 and end < 10 and start < end
                tokens.append(start)
                tokens.append(end)
            else:
                tokens.append(uniform_token)
                tokens.append(uniform_token)

            # TODO: Add different types of mode switch features

            frames = ast.literal_eval(lists[1])
            for f in frames:
                frame_idx = frame_names.index(f) + komo_end_token + 1
                tokens.append(frame_idx)

    tokens.append(komo_end_token)
    return tokens


def komo_from_indices(indices: list[int], frame_names: list[str]) -> str:
    
    text = ""
    feature_list = list(features.keys())
    prev_type = ""
    had_mode_switch = False
    in_mode_switch = False

    uniform_token = max_times + len(feature_list) + len(contraint_types) + 3
    order_token = uniform_token + 1
    order_token_acc = order_token + 1
    objective_start_token = order_token_acc + 1
    mode_switch_token = order_token_acc + 2
    komo_end_token = order_token_acc + 3
    
    for i in indices:
        original_i = i

        if original_i == order_token or original_i == order_token_acc:
            if prev_type != "uniform":
                added = "order="
            else:
                added = ""
            text += f"], {added}{original_i-uniform_token}"
            prev_type = "order"

        if i == komo_end_token:
            break

        ### UNIFORM / TARGET ###
        if i == uniform_token:
            if prev_type == "mode_switch" or prev_type == "komo_start":
                text += "["
            elif prev_type != "uniform":
                text += "], ["
            else:
                text += ", "
            text += f"0.0"
            prev_type = "uniform"
            continue
        
        ### KOMO START ###
        if i == objective_start_token:
            if in_mode_switch:
                text += f"], {not had_mode_switch})\n"    
                had_mode_switch = True
                in_mode_switch = False
            if prev_type == "uniform" or prev_type == "scale":
                text += "])\n"
            elif prev_type == "order":
                text += ")\n"
            text += "komo.addObjective("
            prev_type = "komo_start"
            continue

        ### MODE SWITCH ###
        if i == mode_switch_token:
            if prev_type == "uniform":
                text += "])\n"
            elif prev_type != "":
                text += ")\n"
            text += "komo.addModeSwitch("
            in_mode_switch = True
            prev_type = "mode_switch"
            continue
        
        ### TIMES ###
        if i < max_times:
            if prev_type == "komo_start" or prev_type == "mode_switch":
                text += "["
            elif prev_type == "times":
                text += ", "
            text += f"{i}"
            prev_type = "times"
            continue

        ### FEATURE ###
        i -= max_times
        if i < len(feature_list):
            if prev_type == "times" or prev_type == "uniform":
                text += "], "
            text += f"ry.FS.{feature_list[i]}, "
            prev_type = "feature"
            continue

        ### TYPE ###
        i -= len(feature_list)
        if i < len(contraint_types):
            if prev_type != "frame":
                text += "["
            text += f"], ry.OT.{contraint_types[i]}, "
            prev_type = "type"
            continue

        ### SCALE ###
        i -= len(contraint_types)
        if i < 3:
            if prev_type != "scale":
                text += "["
            else:
                text += ", "
            if i == 0:
                text += f"-1e1"
            elif i == 1:
                text += f"1e1"
            elif i == 2:
                text += f"0"
            prev_type = "scale"
            continue
        
        ### FRAMES ###
        frame_idx = original_i - komo_end_token - 1
        if frame_idx >= 0:
            if in_mode_switch and prev_type != "frame":
                text += "], ry.SY.stable, ["
            elif prev_type != "frame":
                text += "["
            else:
                text += ", "
            text += f"'{frame_names[frame_idx]}'"
            prev_type = "frame"
            continue
        
    if prev_type == "uniform":
        text += "]"

    elif in_mode_switch:
        text += f"], {not had_mode_switch}"
    text += ")\n"

    return text


if __name__ == "__main__":
    text = """komo = ry.KOMO(C, 3, 8, 2, True)
komo.addControlObjective([], 0, 1e-2)
komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e0)
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)


komo.addModeSwitch([0.8, 1.2], ry.SY.stable, ['floor', 'obj2'], False)"""

    frame_names = ['r_gripper', 'obj2', 'floor']
    
    tokens = tokenize_komo(text, frame_names)
    komo_reconstruct = komo_from_indices(tokens, frame_names)
    print(komo_reconstruct)
    print(tokens)
