def komo_error(e: str) -> dict:
    user_message = {
        "role": "user",
        "content": [
            {
                "type": "text",
                "text": f"An error has occurred, try again: {e}.\nRewrite the whole problem again with the appropriate fixes and nothing else.",
            },
        ],
    }
    return user_message


def komo_not_feasible() -> dict:
    user_message = {
        "role": "user",
        "content": [
            {
                "type": "text",
                "text": "The KOMO problem that you have defined seems to not be feasible, please try again.\nRewrite the whole problem again with the appropriate fixes and nothing else.",
            },
        ],
    }
    return user_message


def confirm_image(image_path: str) -> dict:
    user_message = {
        "role": "user",
        "content": [
            {
                "type": "image",
                "image": image_path,
            },
            {
                "type": "text",
                "text": "Looking at this image, would you say that the task has been solved correctly? Start with a simple yes or no, then explain your answer.",
            },
        ],
    }
    return user_message


def not_solved() -> dict:
    user_message = {
        "role": "user",
        "content": [
            {
                "type": "text",
                "text": "Retry generating the KOMO problem as defined previously.\nRewrite the whole problem again with the appropriate fixes and nothing else.",
            },
        ],
    }
    return user_message


def basic_struct(role: str, text: str) -> dict:
    message = {
        "role": role,
        "content": [
            {
                "type": "text",
                "text": text,
            },
        ],
    }
    return message
