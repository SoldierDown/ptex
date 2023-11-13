from PIL import Image

def convert_iff_to_png(input_path, output_path):
    try:
        # Open the IFF file
        with Image.open(input_path) as im:
            # Save as PNG
            im.save(output_path, 'PNG')
        print(f"Conversion successful: {input_path} -> {output_path}")
    except Exception as e:
        print(f"Error during conversion: {e}")

# Replace 'input.iff' and 'output.png' with your file names
convert_iff_to_png('mixtShape_pp_pd_mask.iff', 'mixtShape_pp_pd_mask.png')
