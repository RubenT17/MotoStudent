
"""
/* **********************************************************************************
 *
 *  Created on: 	14.09.2021
 *  Author: 		Rubén Torres Bermúdez <rubentorresbermudez@gmail.com>
 *  Organism:		Sevilla Racing. Universidad de Sevilla.
 *
 *	Description:
 *		This Phyton source file provides the Sevilla Racing STM32F4x project to monitoring
 *      the electric racing bike data for MotoStudent International Competition.
 *		Sevilla Racing is representing the motorcycle team of Universidad de Sevilla.
 *
 *		Creative Commons (CC) 2021 Rubén Torres Bermúdez
 *
 * **********************************************************************************
*/
"""

# <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Licencia Creative Commons" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />Esta obra está bajo una <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Licencia Creative Commons Atribución-NoComercial-CompartirIgual 4.0 Internacional</a>.

# pip install pyserial
# pip install pyinstaller
# python -m serial.tools.list_ports

# Para installar:cd Ruta_carpeta ---> pyinstaller --windowed --onefile --icon=.\nombre del ico    GUI_STM32.py

# Dar permiso para entrar en puerto (linux): sudo chmod 777 /dev/ttyACM0 ó cambiar permiso de usuario a root en ajustes del parámetro tty y relacionados



import  threading
from tkinter import *
from tkinter import messagebox
from tkinter import filedialog
import serial, time, os, sys, time
from PIL import ImageTk, Image



#Declaración de varaibles globales:
data = list(range(0,20))
flag_ajustes = 0        #Flag ajustes guardados
flag_serial = 1         #Flag poder configurar serial
flag_txt = 0            #Flag recopilar en txt
flag_data = 0           #Flag set datos
estado_root = 0
espera = 10
count_text = 0
count_error = 0
data_save = "0"
print(time.strftime("%d/%m/%Y"))
print(time.strftime("%H:%M:%S"))

n=0


def scanSerial(num_ports, verbose):
    dispositivos_serie = []
    if verbose:
        print ("Escanenado %d puertos serie:" % num_ports)
    for i in range(num_ports):
      if sys.platform.startswith('win'):
        if verbose:
            print("Puerto " + "COM" + f'{i}')
        try:
            s = serial.Serial("COM" + f'{i}')
            if verbose: print ("OK --> %s" % s.portstr)
            dispositivos_serie.append(s.portstr)
            s.close()
        except:
            if verbose: print ("NO")
            pass
        if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            if verbose:
                print("Puerto " + "/dev/ttyACM" + f'{i}')
            try:
                s = serial.Serial("/dev/ttyACM" + f'{i}')
                if verbose: print ("OK --> %s" % s.portstr)
                dispositivos_serie.append(s.portstr)
                s.close()
            except:
                if verbose: print ("NO")
                pass
    return dispositivos_serie



def Serial_STM():
    global count_error, flag_data, flag_ajustes, flag_serial
    try:
        b = stm32.readline()
    except:
        count_error +=1
        b=b'0'
        if count_error==20:
            print ("ERROR DE COMUNICACIÓN. REINICIA EL AJUSTE DE PUERTO")
            stm32.close()
            flag_data=0
            flag_ajustes = 0
            flag_serial = 1
    #print(b)
    while b == b'+++++\r\n':
        #print("COMIENZA LA CADENA")
        for i in range (0, 20):
            data[i] = stm32.readline()
            if data[i] == b'*****\r\n':
                return "COMUNICACIÓN OK"
            data[i]=(data[i].decode()).rstrip('\r\n')
            #print(data[i])
    return "NO COMUNICACIÓN"


def Data():
    global espera, flag_data
    IMD_freq.set(data[0])
    IMD_duty.set(data[1])
    AcellX.set(data[2])
    AcellY.set(data[3])
    AcellZ.set(data[4])
    GyroX.set(data[5])
    GyroY.set(data[6])
    DS_temp1.set(data[7])
    DS_temp2.set(data[8])
    ADC_ch1.set(data[9])
    ADC_ch2.set(data[10])
    ADC_ch3.set(data[11])
    ADC_ch4.set(data[12])
    ADC_temp.set(data[13])
    flag_data=0
    tiempo = time.time()
    if flag_txt==1 and tiempo>espera:
        escribir_txt()
        espera = tiempo+10


def escribir_txt():
    global count_text
    count_text+=1
    txt.write("###### INICIO DE DATOS ######     " + time.strftime("%d/%m/%Y") + "\n")
    txt.write("Hora: "+time.strftime("%H:%M:%S")+ "   Transmisión Nº: "+f"{count_text}" +
                "   Puerto: "+f"{puertof.get()}" "   Tasa: " +f"{baudiof.get()}"+ "\n")
    txt.write("Frecuencia PWM: " + f"{data[0]}" + "Hz\n")
    txt.write("Ciclo de trabajo PWM: " +f"{data[1]}" + "%\n")
    txt.write("Aceleración X: " + f"{data[2]}" + "g\n")
    txt.write("Aceleración Y: " + f"{data[3]}" + "g\n")
    txt.write("Aceleración Z: " + f"{data[4]}" + "g\n")
    txt.write("Inclinación X: " + f"{data[5]}" + "º\n")
    txt.write("Inclinación Y: " + f"{data[6]}" + "º\n")
    txt.write("Temp. pack 1: " + f"{data[7]}" + "ºC\n")
    txt.write("Temp. pack 2: " + f"{data[8]}" + "ºC\n")
    txt.write("ADC Canal 1: " + f"{data[9]}" + "\n")
    txt.write("ADC Canal 2: " + f"{data[10]}" + "\n")
    txt.write("ADC Canal 3: " + f"{data[11]}" + "\n")
    txt.write("ADC Canal 4: " + f"{data[12]}" + "\n")
    txt.write("ADC Temp uC: " + f"{data[13]}" + "ºC\n")
    txt.write("###### FIN DE DATOS ######\n\n\n")


def Thread_Data():
    while flag_serial==0 :
        #a = Serial_STM()
        #print(a)
        Serial_STM()







#################################          INICIO DE TKINTER          #################################
root = Tk() # Creamos la raiz o root de nuestra app
estado_root=1
root.title("Serial STM32 Sevilla Racing")
#root.iconbitmap("python_icone.ico")
root.resizable(1, 1)
root.geometry("1500x600+0+150")#Estaba en 900x600+300+150
root.configure(bg=None, bd=0)

def cerrarroot():
    global estado_root, txt
    if messagebox.askokcancel("Salir", "¿Seguro que desea salir?"):
        estado_root=0
root.protocol("WM_DELETE_WINDOW", cerrarroot)


#Declaración de variables
IMD_freq = StringVar(value=data[0])
IMD_duty = StringVar(value=data[1])
AcellX = StringVar(value=data[2])
AcellY = StringVar(value=data[3])
AcellZ = StringVar(value=data[4])
GyroX = StringVar(value=data[5])
GyroY = StringVar(value=data[6])
DS_temp1 = StringVar(value=data[7])
DS_temp2 = StringVar(value=data[8])
ADC_ch1 = StringVar(value=data[9])
ADC_ch2 = StringVar(value=data[10])
ADC_ch3 = StringVar(value=data[11])
ADC_ch4 = StringVar(value=data[12])
ADC_temp = StringVar(value=data[13])
puerto = StringVar()
puertof = StringVar()
baudio = StringVar()
baudiof = StringVar()
velocidad = StringVar()



#Frame Principal
frame = Frame(root, bd=0)
frame.pack(fill="both", expand=True)
frame.config (bg="#014a91", cursor="diamond_cross") #diamond_cross, circle, dot
img=PhotoImage(format="png", file=os.getcwd()+"/bin/SevillaRacing2.png")

labelimage = Label(frame, image=img)
labelimage.place(x=0, y=0)



#Título
title = Label(frame, text="INFORMACIÓN SERIAL DEL STM32", font=("Comic Sans MS", 16), bg="#014a91", fg="white", justify="center")
title.place(x=260, y=15)
#title.grid(row=0, column=0, columnspan=5000, padx=20, pady=20)


#Frame de IMD
IMD_frame = Frame(frame)
IMD_frame.place(x=10, y=90)
#IMD_frame.grid(row=2, column=0, padx=10, pady=10, sticky='NW')
IMD_frame.config (bg="#014a91", cursor="diamond_cross")
IMD_label = Label (IMD_frame, text="Insulation Monitorig Device (IMD)", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="center")
IMD_label.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

freq_label = Label (IMD_frame, text="Frecuencia PWM:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
freq_label.grid(row=1, column=0, padx=1, pady=2, sticky='W')
freq_label.config (bg="#014a91", cursor="diamond_cross")
freq_entry = Entry(IMD_frame, textvariable=IMD_freq, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
freq_entry.grid (row=1, column=1, padx=1, pady=2, sticky='W')

duty_label = Label (IMD_frame, text="Ciclo de trabajo PWM:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
duty_label.grid(row=2, column=0, padx=1, pady=2, sticky='W')
duty_label.config (bg="#014a91", cursor="diamond_cross")
duty_entry = Entry(IMD_frame, textvariable=IMD_duty, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
duty_entry.grid (row=2, column=1, padx=1, pady=2, sticky='W')


#Frame de MPU6050
MPU_frame = Frame(frame)
MPU_frame.place(x=650, y=220)
#MPU_frame.grid(row=3, column=0, padx=10, pady=10, sticky='NW')
MPU_frame.config (bg="#014a91", cursor="diamond_cross")
MPU_label = Label (MPU_frame, text="MPU6050", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="center")
MPU_label.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

AcellX_label = Label (MPU_frame, text="Aceleración X:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
AcellX_label.grid(row=1, column=0, padx=1, pady=2, sticky='W')
AcellX_label.config (bg="#014a91", cursor="diamond_cross")
AcellX_entry = Entry(MPU_frame, textvariable=AcellX, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
AcellX_entry.grid (row=1, column=1, padx=1, pady=2, sticky='W')

AcellY_label = Label (MPU_frame, text="Aceleración Y:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
AcellY_label.grid(row=2, column=0, padx=1, pady=2, sticky='W')
AcellY_label.config (bg="#014a91", cursor="diamond_cross")
AcellY_entry = Entry(MPU_frame, textvariable=AcellY, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
AcellY_entry.grid (row=2, column=1, padx=1, pady=2, sticky='W')

AcellZ_label = Label (MPU_frame, text="Aceleración Z:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
AcellZ_label.grid(row=3, column=0, padx=1, pady=2, sticky='W')
AcellZ_label.config (bg="#014a91", cursor="diamond_cross")
AcellZ_entry = Entry(MPU_frame, textvariable=AcellZ, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
AcellZ_entry.grid (row=3, column=1, padx=1, pady=2, sticky='W')

GyroX_label = Label (MPU_frame, text="Inclinación X:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
GyroX_label.grid(row=4, column=0, padx=1, pady=2, sticky='W')
GyroX_label.config (bg="#014a91", cursor="diamond_cross")
GyroX_entry = Entry(MPU_frame, textvariable=GyroX, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
GyroX_entry.grid (row=4, column=1, padx=1, pady=2, sticky='W')

GyroY_label = Label (MPU_frame, text="Inclinación Y:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
GyroY_label.grid(row=5, column=0, padx=1, pady=2, sticky='W')
GyroY_label.config (bg="#014a91", cursor="diamond_cross")
GyroY_entry = Entry(MPU_frame, textvariable=GyroY, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
GyroY_entry.grid (row=5, column=1, padx=1, pady=2, sticky='W')


#Frame de DS18B20
DS_frame = Frame(frame)
DS_frame.place(x=650, y=90)
#DS_frame.grid(row=2, column=9900, padx=10, pady=10, sticky='NE')
DS_frame.config (bg="#014a91", cursor="diamond_cross")
DS_label = Label (DS_frame, text="Temperaturas", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="center")
DS_label.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

temp1_label = Label (DS_frame, text="Temp. pack 1:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
temp1_label.grid(row=1, column=0, padx=1, pady=2, sticky='W')
temp1_label.config (bg="#014a91", cursor="diamond_cross")
temp1_entry = Entry(DS_frame, textvariable=DS_temp1, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
temp1_entry.grid (row=1, column=1, padx=1, pady=2, sticky='W')

temp2_label = Label (DS_frame, text="Temp. pack 2:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
temp2_label.grid(row=2, column=0, padx=1, pady=2, sticky='W')
temp2_label.config (bg="#014a91", cursor="diamond_cross")
temp2_entry = Entry(DS_frame, textvariable=DS_temp2, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
temp2_entry.grid (row=2, column=1, padx=1, pady=2, sticky='W')


#Frame de ADC
ADC_frame = Frame(frame)
ADC_frame.place(x=10, y=220)
#ADC_frame.grid(row=5, column=200, padx=10, pady=10, sticky='NW')
ADC_frame.config (bg="#014a91", cursor="diamond_cross")
ADC_label = Label (ADC_frame, text="Analogue to Digital Converter (ADC)", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="center")
ADC_label.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

ch1_label = Label (ADC_frame, text="Canal 1:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
ch1_label.grid(row=1, column=0, padx=1, pady=2, sticky='W')
ch1_label.config (bg="#014a91", cursor="diamond_cross")
ch1_entry = Entry(ADC_frame, textvariable=ADC_ch1, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
ch1_entry.grid (row=1, column=1, padx=1, pady=2, sticky='W')

ch2_label = Label (ADC_frame, text="Canal 2:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
ch2_label.grid(row=2, column=0, padx=1, pady=2, sticky='W')
ch2_label.config (bg="#014a91", cursor="diamond_cross")
ch2_entry = Entry(ADC_frame, textvariable=ADC_ch2, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
ch2_entry.grid (row=2, column=1, padx=1, pady=2, sticky='W')

ch3_label = Label (ADC_frame, text="Canal 3:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
ch3_label.grid(row=3, column=0, padx=1, pady=2, sticky='W')
ch3_label.config (bg="#014a91", cursor="diamond_cross")
ch3_entry = Entry(ADC_frame, textvariable=ADC_ch3, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
ch3_entry.grid (row=3, column=1, padx=1, pady=2, sticky='W')

ch4_label = Label (ADC_frame, text="Canal 4:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
ch4_label.grid(row=4, column=0, padx=1, pady=2, sticky='W')
ch4_label.config (bg="#014a91", cursor="diamond_cross")
ch4_label = Entry(ADC_frame, textvariable=ADC_ch4, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
ch4_label.grid (row=4, column=1, padx=1, pady=2, sticky='W')

temp_label = Label (ADC_frame, text="Temp. uC:", font=("Comic Sans MS", 11), bg="#014a91", fg="white", justify="left")
temp_label.grid(row=5, column=0, padx=1, pady=2, sticky='W')
temp_label.config (bg="#014a91", cursor="diamond_cross")
temp_label = Entry(ADC_frame, textvariable=ADC_temp, font=("Comic Sans MS", 11), bg="#014a91", fg="black", justify="left", width = 8, state="readonly")
temp_label.grid (row=5, column=1, padx=1, pady=2, sticky='W')



##############     MENÚ     ##############
menu = Menu(root)
root.config(menu=menu)


def ventana_ajustes():
    rootajuste = Tk()
    rootajuste.title("Serial STM32 Sevilla Racing")
    rootajuste.resizable(0, 0)
    rootajuste.geometry("300x180+400+250")
    rootajuste.configure(bg="#014a91")
    frameajuste = Frame(rootajuste)
    frameajuste.configure(bg="#014a91", cursor="diamond_cross") #diamond_cross, circle, dot
    frameajuste.pack(fill="both", expand=True)

    Label(frameajuste, text="Elegir puerto COM/ACM:", font=("Comic Sans MS", 11), bg="#014a91", fg="white").place(x=16, y= 30)
    ports = scanSerial(20, True)
    if ports: pass
    else:
        ports.append("----")
    portmenu=OptionMenu(frameajuste, puerto, *ports)
    portmenu.configure(bg="#014a91")
    portmenu.place(x=210, y=25)
    puerto.get()

    baudlabel=Label(frameajuste, text="Elegir tasa de baudios:", font=("Comic Sans MS", 11), bg="#014a91", fg="white").place(x=16, y= 80)
    baudrate = ["9600", "115600", "1000000", "2000000"]
    baudmenu=OptionMenu(frameajuste, baudio, *baudrate)
    baudmenu.configure(bg="#014a91")
    baudmenu.place(x=200, y= 75)
    baudio.get()

    def Ajustes_save():
        global flag_ajustes, flag_serial
        if (puerto.get()=="----" or puerto.get()==""):
            print("No se ha seleccionado el puerto")
            messagebox.showwarning("Fallo COM", "No se ha seleccionado el puerto")
            flag_ajustes = 0
        elif (baudio.get()=="----"or baudio.get()==""):
            print("No se ha seleccionado la tasa de baudios")
            messagebox.showwarning("Fallo en baudios", "No se ha seleccionado la tasa de baudios")
            flag_ajustes = 0
        else:
            flag_ajustes = 1
            flag_serial = 1
            puertof.set(puerto.get())
            baudiof.set(baudio.get())
            print(puerto.get())
            print(baudiof.get())
            rootajuste.destroy()
    Button(frameajuste, text="Guardar", command=Ajustes_save, bg="#ffed00").place(x=180, y= 140)

    def Ajustes_exit():
        rootajuste.destroy()
    Button(frameajuste, text="Salir", command=Ajustes_exit, bg="#ffed00").place(x=260, y= 140)

def ventana_salir():
    global estado_root
    if messagebox.askokcancel("Salir", "¿Seguro que desea salir?"):
        estado_root=0

def crear_txt():
    global txt, flag_txt
    #txt=open("Datos_STM32_"+time.strftime("%d%m%Y")+".txt", "w")
    txt=open(ruta, "w")
    flag_txt = 1


def avisoguardar():
    try:
        crear_txt()
        messagebox.showinfo("Recopilar en...", "Se están recopilando los datos.")
        print("La recopilación se hará en: " + ruta)
    except: pass


def Recopilaren():
    global ruta
    if flag_txt==0:
        ruta=filedialog.asksaveasfilename(title="Recopilar en...", filetypes=(("Ficheros de texto", "*txt"), ("Todos los ficheros", "*.*")))
        avisoguardar()
    else:
        messagebox.showinfo("Recopilar en...", "Ya se están recopilando los datos.")

archivo=Menu(menu, tearoff=0)
archivo.add_command(label="Nuevo")
archivo.add_command(label="Recopilar en...", command=Recopilaren)
archivo.add_separator()
archivo.add_command(label="Ajustes", command=ventana_ajustes)
archivo.add_separator()
archivo.add_command(label="Salir", command=ventana_salir)


def ventana_acercade():
    rootacercade = Tk()
    rootacercade.title("Acerca de...")
    rootacercade.resizable(0,0)
    rootacercade.geometry("400x300+400+250")
    rootacercade.configure(bg="#014a91")
    frameacercade = Frame(rootacercade)
    frameacercade.configure(bg="#014a91", cursor="diamond_cross") #diamond_cross, circle, dot
    frameacercade.pack(fill="both", expand=True)
    texto = "\n\nEsta aplicación fue diseñada por Rubén Torres Bermúdez para el monitoreo de datos del STM32 del proyecto MotoStudent 2022 del equipo Sevilla Racing de la Universidad de Sevilla.\r\n\nCreative Commons (CC) 2021 Rubén Torres Bermúdez"
    Message(frameacercade, font=("Comic Sans MS", 11), bg="#014a91", fg="#ffed00", text=texto).pack()


def ventana_info():
    rootinfo = Tk()
    rootinfo.title("Información de uso")
    rootinfo.resizable(0,0)
    rootinfo.geometry("300x200+400+250")
    rootinfo.configure(bg="#014a91")
    frameinfo = Frame(rootinfo)
    frameinfo.configure(bg="#014a91", cursor="diamond_cross") #diamond_cross, circle, dot
    frameinfo.pack(fill="both", expand=True)
    texto = "\n\nPara usar esta aplicación es necesario entrar en Archivo --> Ajustes y seleccionar uno de los puertos y una tasa de baudios disponible. Después de esto, presionar guardar y comenzará a visualizarse los datos del Serial seleccionado"
    Message(frameinfo, font=("Comic Sans MS", 11), bg="#014a91", fg="#ffed00", text=texto).pack()


ayuda=Menu(menu, tearoff=0)
ayuda.add_command(label="Acerca de...", command=ventana_acercade)
ayuda.add_command(label="Información", command=ventana_info)

menu.add_cascade(label="Archivo", menu=archivo)     #Creamos la cascada de elementos visibles. IMPORTANTE:label, en minúsculas
menu.add_cascade(label="Ayuda", menu=ayuda)





#Canvas
speedometer=Canvas(frame, width=600, height=600, bg="#014a91", bd=0)
speedometer.place(x=900, y=0)
meter=PhotoImage(format="png", file=os.getcwd()+"/bin/meter1.png")
speedometer.create_image(300,300,image=meter)
needle=Image.open(os.getcwd()+"/bin/needle1.png")

a = speedometer.create_text(300,450, font=("My Display St", 24), fill="#ffed00")
# Entry(frame, font=("My Display St", 24), fg="#ffed00", bg="black", textvariable=velocidad, width = 6, bd=0).place(x=900+300-50, y=300+150)







#################################          FIN DE TKINTER          #################################
asas=0
while estado_root:
    if(flag_ajustes==1):
        if (flag_ajustes==1 and flag_serial==1):
            stm32 = serial.Serial(puertof.get(), baudiof.get(), timeout=0.005)
            stm32.write(b"P")
            SerialRx = threading.Thread(target=Thread_Data).start()
            flag_serial=0

        Data()
        # stm32.write(b"P")
        try:
            if not data[1]=="":
                rotated_needle =ImageTk.PhotoImage(needle.rotate(250-3*float(data[1])))   #250 = 0m/s
                data_save=data[1]
        except:
            print("ERROR DATO VELOCIDAD")
            pass
        speedometer.create_image(300,300,image=rotated_needle)
        speedometer.itemconfig(a, text="{:.0f}".format(3*float(data[1])))  #"{:.20f}".format(3*float(data[1]))
    
    # Prueba para ver las letras
    asas =asas+1
    velocidad.set(f'{asas}')
    time.sleep(0.05)

    root.update()



try:
    root.destroy()
except: pass

try:
    txt.close()
except: pass

try:
    stm32.write(b"C")
    stm32.close()
except: pass