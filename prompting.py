from qwen_vl_utils import process_vision_info


def prompt_qwen(model, processor, device: str, task_description: str, image_path: str):

    with open("./prompts/komo_tutorial.txt", 'r') as file:
        komo_tutorial = file.read()

    with open("./prompts/problem_definition.txt", 'r') as file:
        problem_definition = file.read()

    messages = [
        {
            "role": "user",
            "content": [
                {
                    "type": "text",
                    "text": komo_tutorial,
                },
                {
                    "type": "image",
                    "image": image_path,
                },
                {
                    "type": "text",
                    "text": f"{problem_definition}{task_description}",
                },
            ],
        }
    ]

    # Preparation for inference
    text = processor.apply_chat_template(
        messages, tokenize=False, add_generation_prompt=True
    )
    image_inputs, video_inputs = process_vision_info(messages)
    inputs = processor(
        text=[text],
        images=image_inputs,
        videos=video_inputs,
        padding=True,
        return_tensors="pt",
    )
    inputs = inputs.to(device)

    # Inference: Generation of the output
    generated_ids = model.generate(**inputs, max_new_tokens=1024)
    generated_ids_trimmed = [
        out_ids[len(in_ids) :] for in_ids, out_ids in zip(inputs.input_ids, generated_ids)
    ]
    output_text = processor.batch_decode(
        generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False
    )

    komo_definition = output_text[0]

    return komo_definition
