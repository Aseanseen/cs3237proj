import base64

# Serializes image from png to string
def get_base64string_from_img_path(image_path):
    try:
        with open(image_path, mode="rb") as file:
            img = file.read()
        return base64.b64encode(img).decode("utf-8")  # picture to bytes, then to string
    except:
        return None


def get_base64_from_utf8(encoded_str):
    result = encoded_str.encode("utf-8")
    return base64.b64decode(result)


# Deserializes image from string to png, then save it in the specified file directory
def get_img_path_from_base64string(encoded_str, image_path):
    try:
        result = get_base64_from_utf8(encoded_str)
        # create a writable image and write the decoding result
        with open(image_path, "wb") as image:
            image.write(result)
    except:
        return None