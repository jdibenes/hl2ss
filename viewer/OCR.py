def text_detect(pos_thumb, pos_index, model, image):
    xmin = min(pos_index[1], pos_thumb[1])
    xmax = max(pos_index[1], pos_thumb[1])
    ymin = min(pos_index[0], pos_thumb[0])
    ymax = max(pos_index[0], pos_thumb[0])
    print(xmin, ymin, ymax, xmax)
    if (xmin >= 0 and ymin >= 0 and  xmax < image.shape[1] and ymax < image.shape[0]):                
        doc = image[ymin:ymax, xmin:xmax]
        print(model(doc))
