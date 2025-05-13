function [coste, ruta_final] = Dijkstra(Matriz, origen, destino)
    ruta = [];
    vector = (1:size(Matriz))';
    Exploracion = [vector, [Inf(size(Matriz,1),1)], zeros(size(Matriz,1),1)];
    Exploracion(origen,2) = 0;

    while(size(Exploracion,1)>=1)
        Exploracion = sortrows(Exploracion,2);
        nodo_actual = Exploracion(1,1);%El nodo actual, para mirar en la fila correcta de Matriz
 
        for j=1:size(Matriz,1) %Recorro todas las columnas de la Matriz para la fila nodo_actual
            if(Matriz(nodo_actual,j)~=0) %Anoto el coste de nodos alcanzables
                fila_j = find(Exploracion(:,1) == j);
                if ~isempty(fila_j)
                    suma_coste=Exploracion(1,2)+Matriz(nodo_actual,j);%Suma del padre(Exploracion,donde estoy) acumulado y del nodo objetivo(Matriz(j,2))
                    if(suma_coste<Exploracion(fila_j,2))%Si dicha suma es menor que la que ya estaba registrada para ese nodo(Exploracion(Matriz(j,1))), entonces se guarda.
                        Exploracion(fila_j,2) = suma_coste;%coste menor
                        Exploracion(fila_j,3) = nodo_actual;%El nodo padre, es el nodo actual en el que estamos
                    end
                end
            end
        end
    
    ruta = [ruta; Exploracion(1,:)]; %Guardo este nodo como parte de la ruta
    Exploracion = Exploracion([2:end],:);%Elimino la primera fila, que es el nodo que he guardado para seguir buscando
    if nodo_actual == destino
        break;
    end
    coste = Exploracion(1,2);%Despues del wile, solo quedara una fila, y su coste sera el de la ruta entera
    end
%%RECONSTRUCCION DE LA RUTA
    nodo_actual = destino;
    i = size(ruta,1);
    ruta_final = [destino];
    while(nodo_actual~=origen)
        fila_i = find(ruta(:,1) == nodo_actual);
            if ruta(fila_i,3) == ruta(i,1)
                ruta_final = [ruta(i,1);ruta_final];
                nodo_actual = ruta(i,1);
            else
                i = i-1;
            end
    end
end
