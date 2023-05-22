clc;
clear all;
close all;

carpeta = 'C:\Users\joel1\OneDrive\Escritorio\TIV'; %colocar la ruta en donde se encuntra el archivo
c= pi/180;
radio = 6378.1;

disp(["*********Calculo de las variables de la E-bike***********"]);
Einstr = input("Colocar la energia de entrada del sistema (kWh): ", 's');

mensaje = 'Seleccionar archivo de coordenadas.';
titulo = 'Coordenadas';
boton_presionado = questdlg(mensaje, titulo, 'OK', 'Cancelar', 'OK');

if strcmp(boton_presionado, 'OK')
    [archivo, ruta] = uigetfile(fullfile(carpeta, '*.*'), 'Selecciona un archivo');

    if archivo ~= 0
        name_coordenadas = archivo;
    end

end

mensaje_po = 'Seleccionar archivo de potencia.';
titulo_po = 'Potencia';
boton_presionado2 = questdlg(mensaje_po, titulo_po, 'OK', 'Cancelar', 'OK');

if strcmp(boton_presionado2, 'OK')
    [archivo2, ruta] = uigetfile(fullfile(carpeta, '*.*'), 'Selecciona un archivo');

    if archivo2 ~= 0
        name_power = archivo2;
    end

end

fid = fopen(name_power, 'r');
fid_csv = textscan(fid, '%s%f', 'delimiter', ';', 'HeaderLines', 1);
time = fid_csv{1};
fclose(fid);

Ein = str2double(Einstr);

time_num = datenum(time, 'yyyy-mm-dd HH:MM:SS');
time_seconds = ((time_num - time_num(1)) * 86400);
time_seconds = transpose(time_seconds);
dis = dlmread(name_coordenadas, ';');
pow = dlmread(name_power, ';');


sz = size(dis);
latitude_start = dis(2,2);
longitude_start = dis(2,3);
distance_Total = 0;
d = [];
d(1)=0;
for i = 3:sz(1)
  deltalatitude = dis(i,2) - latitude_start;
  deltalongitude = dis(i,3) - longitude_start;
  a = sin(deltalatitude * c / 2) .^ 2 + cos(latitude_start * c) * cos(dis(i,2) * c) * sin(deltalongitude * c / 2) .^ 2;
  distance = (2 * asin(sqrt(a)))* radio * 1000;
  d(i-1) = distance;
  distance_Total = distance + distance_Total;
  latitude_start = dis(i,2);
  longitude_start = dis(i,3);
end



power = (pow(2:end,2)/30)*36;
energy = trapz(time_seconds,power)/(1000*3600);
effi = (energy/Ein)*100;
distance_Total = distance_Total/1000;
distance_Total_millas = distance_Total/1.609344;
effi_millas = (energy/distance_Total_millas)*100;

cost = energy * 0.092;

disp(["Distancia: " num2str(distance_Total) " Km"]);
disp(["Eficiencia Energetica: " num2str(effi) " %"]);
disp(["Energia: " num2str(energy) "kWh"]);
disp(["Eficiencia Energetica per millas: " num2str(effi_millas) " kWh-per milla"]);
disp(["Costo viaje: " num2str(cost) "USD"]);

subplot(2,3,1)
plot(time_seconds,d);
title('Distancia recorrida')
xlabel('tiempo[s]')
ylabel('Distancia [m]')


subplot(2,3,2)
bar(1, distance_Total);
title('Distancia Total')
ylabel('Distancia[Km]')
text(1, distance_Total+1, num2str(distance_Total))

subplot(2,3,3)
plot(time_seconds,power);
title('Potencia')
xlabel('Tiempo[s]')
ylabel('P[W]')

subplot(2,3,4)
bar(1, cost);
title('Costo')
ylabel('Precio[$]')
text(1, cost+0.001, num2str(cost))

subplot(2,3,5)
bar(1, effi);
title('Eficiencia Energetica')
ylabel('%')
text(1, effi+2, num2str(effi))

subplot(2,3,6)
bar(1, effi_millas);
title('Eficiencia Energetica')
ylabel('n[kWh per 100 miles]')
text(1, effi_millas+0.1, num2str(effi_millas))
