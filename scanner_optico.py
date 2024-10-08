import cv2
import numpy as np
import serial as serial
import time
import math

class Camera:
    def __init__(self, camera_index=1): # Variável camera_index indica a "webcam" a ser acessada, 0 webcam nativa, 1 adicional.
        self.cam = cv2.VideoCapture(camera_index)
        self.frames_list = []

    def capture_frame(self):
        validacao, frame = self.cam.read()
        if validacao:
            cv2.imshow("Video da Webcam", frame)
            return frame
        else:
            return None
        
    def show_image1x1(self, margin=20): # A imagem inicial é 480x640 e a final é 440x440
        validacao, frame = self.cam.read()
        if validacao:
            # Obtém as dimensões da imagem
            altura, largura = frame.shape[:2]
            nova_altura = altura - 2 * margin
            nova_largura = largura - 2 * margin
            # Garante que as novas dimensões sejam válidas
            if nova_altura > 0 and nova_largura > 0:
                # Corta a margem de cada lado
                frame_cortado = frame[margin:altura - margin, margin:largura - margin]
                menor_dimensao = min(nova_altura, nova_largura) # Determina a menor dimensão
                frame_cortado = frame_cortado[:menor_dimensao, :menor_dimensao] # Ajusta a imagem para que as dimensões sejam iguais à menor dimensão
                return frame_cortado
            else:
                print("Margem muito grande para as dimensões da imagem.")
                return None
        else:
            return None

    def save_frame(self, frame, img_name):
        path = r"C:\Users\caiod\OneDrive\Documentos\Projetos Python\IC\Imagens Finais\Teste\\" + img_name
        cv2.imwrite(path, frame)

    def calculate_mean_frame(self):
        if self.frames_list:
            mean_frame = np.mean(self.frames_list, axis=0).astype(np.uint8)
            return mean_frame

    def increase_saturation(self, frame, saturation_scale=1.5):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * saturation_scale, 0, 255)
        saturated_frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return saturated_frame
    
    def capture_images(self, frames_max, X_index=None, Y_index=None):
        time.sleep(1) # Aguardar estabilizar a imagem
        self.frames_list = []
        while frames_max > len(self.frames_list):
            time.sleep(0.1) # Garantir que diferentes frames sejam utilizados no cálculo da média
            frame = self.show_image1x1()
            if frame is not None:
                frame = camera.increase_saturation(frame)
                cv2.imshow("Frame", frame)  # Exibe a imagem processada
                cv2.waitKey(5) & 0xFF
                self.frames_list.append(frame)
        if self.frames_list:  # Verifica se a lista de frames não está vazia
            mean_frame = self.calculate_mean_frame()
            if mean_frame is not None:
                if X_index is not None and Y_index is not None:
                    self.save_frame(mean_frame, "X_{}-Y_{}.png".format(X_index, Y_index))
            else:
                print("Erro ao calcular a média dos frames.")
        else:
            print("Nenhum frame foi capturado.")    

    def release(self):
        self.cam.release()
        cv2.destroyAllWindows()

ser = serial.Serial(
        port = 'COM5', #Verificar a porta
        bytesize = 8,
        parity = 'N', 
        stopbits = 1,
        baudrate = 115200,
        timeout = 10,
        xonxoff = False
)
ser.reset_output_buffer()
ser.reset_input_buffer()
val = 0

if __name__ == "__main__":
    camera = Camera()
    frames_max = 5
    x = float(input("Informe a dimensão alocada no eixo x (mm): "))
    y = float(input("Informe a dimensão alocada no eixo y (mm): "))
    print("Passo foi definido como 2.33mm")
    passo = 2.33
    passo_str = str(passo)
    
    y_div_passo = math.ceil(y / passo) # A função math.ceil() arredonda para cima o valor de y / passo
    x_div_passo = math.ceil(x / passo)

    # Definir o modo de posicionamento absoluto
    command = 'G90 \r\n'  # Modo de posicionamento absoluto
    val = ser.write(command.encode(encoding='ascii', errors='strict'))

    command = 'G21 \r\n'  # Define a unidade de medida como milímetros
    val = ser.write(command.encode(encoding='ascii', errors='strict'))

    command = 'G92 X0 Y0 \r\n'  # Define a posição atual como origem (0,0)
    val = ser.write(command.encode(encoding='ascii', errors='strict'))

    # Inicializar a posição atual
    pos_x = 0.00
    pos_y = 0.00

    for i in range(y_div_passo + 1):
        for j in range(x_div_passo + 1):
            if i % 2 == 0:  # Movimento da esquerda para a direita na grade
                pos_x = j * passo
            else:  # Movimento da direita para a esquerda na grade
                pos_x = (x_div_passo - j) * passo
            
            # Mover para a nova posição
            command = f'G0 X{pos_x} Y{pos_y} \r\n'
            val = ser.write(command.encode(encoding='ascii', errors='strict'))
            camera.capture_images(frames_max, round(pos_x, 2), round(pos_y, 2))
        
        # Mover para a próxima linha na grade
        pos_y += passo  # Atualizar a posição Y para a próxima linha
        
    # Após completar a leitura da folha, mover a máquina de volta para a posição inicial
    command = 'G0 X0 Y0 \r\n'  # Mover para (0,0)
    val = ser.write(command.encode(encoding='ascii', errors='strict'))

    camera.release()