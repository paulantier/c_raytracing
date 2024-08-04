import cv2
import pythonPNG2PPM as pn2pp

for i in range(100):
    pn2pp.ppm_to_jpg("rendus/rendu_"+str(i)+".ppm","rendus/rendu_"+str(i)+".jpg")

# Lecture de la première image pour obtenir les dimensions
premiere_image = cv2.imread("rendus/rendu_0.jpg")
hauteur, largeur, _ = premiere_image.shape

# Spécifications pour la création de la vidéo
nom_video = 'video_Sphere.avi'  # Nom de la vidéo de sortie
fps = 5  # Images par seconde (fps)

# Initialisation de l'objet vidéo
video = cv2.VideoWriter(nom_video, cv2.VideoWriter_fourcc(*'XVID'), fps, (largeur, hauteur))

# Parcours des images et ajout à la vidéo
for i in range(100):
    chemin_image = "rendus/rendu_"+str(i)+".jpg"
    image = cv2.imread(chemin_image)
    video.write(image)

# Fermeture de l'objet vidéo
video.release()

print("Vidéo créée avec succès : ", nom_video)
