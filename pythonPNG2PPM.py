from PIL import Image
import numpy as np

def create_PPM():
    chemin_image = 'terre.jpg'  # Remplace avec le chemin de ton image
    image = Image.open(chemin_image)
    image = np.array(image)
    return image

def write_ppm(image, filename):
    height, width, depth = image.shape
    with open(filename, 'w') as f:
        f.write('P3\n')
        f.write(''+str(width)+' '+str(height)+' ')
        f.write('255\n')
        for y in range(height):
            for x in range(width):
                r, g, b = (image[y, x,0]), (image[y, x,1]),(image[y, x,2])
                f.write(''+str(r)+' ')
                f.write(''+str(g)+' ')
                f.write(''+str(b)+'\n')


#write_ppm(create_PPM(),'terre.ppm')

def read_ppm(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        width, height = map(int, lines[1].split())
        data = []
        for line in lines[3:]:
            data.extend(map(int, line.split()))
    return np.array(data).reshape(height, width, 3)

def ppm_to_jpg(input_file, output_file):
    try:
        image_data = read_ppm(input_file)
        img = Image.fromarray(image_data.astype('uint8'))
        img.save(output_file, 'JPEG')
        print("Conversion terminée avec succès !")
    except Exception as e:
        print("Une erreur est survenue lors de la conversion :", str(e))

input_ppm_file = 'rendus/rendu2024-01-10_14-18-28.ppm'
output_jpg_file = 'rendu3.jpg'

ppm_to_jpg(input_ppm_file, output_jpg_file)
