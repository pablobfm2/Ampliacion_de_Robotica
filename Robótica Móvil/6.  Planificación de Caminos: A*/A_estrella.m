function [coste,ruta]=A_estrella(G,H,inicio,fin)

%Se añade la matriz H que contiene la heurística
n=length(G);
n_cercanos=[(1:n)' inf(n,1) inf(n,1) zeros(n,1)]; %Vector de infinitos para los costes y para la heurística que iremos cambiando según exploremos
index=inicio; %La primera linea a analizar siempre será el nodo donde estamos 
n_cercanos(index,2)=0; %Ponemos el coste a 0 en el nodo donde estamos 
n_sacados=[];

for i=1:n
    for j=1:n 
        nuevo_coste=n_cercanos(index,2)+G(index,j);
        nueva_heuristica=nuevo_coste+H(j,fin); 
        if(G(index,j)~=0)
            if nueva_heuristica < n_cercanos(j,3) %Para actualizar el valor en caso que en el nuevo camino se llegue con menos coste
                n_cercanos(j,2)=nuevo_coste; %Guardamos datos de los nodos que estan contiguos al actual
                n_cercanos(j,3)=nueva_heuristica;
                n_cercanos(j,4)=index;
            end 
        end
    end
    %Marcar el nodo actual como visitado añadiendolo a n_sacados
    n_sacados=[n_sacados;n_cercanos(index,:)];

    if(index==fin)
       break; 
    end

    %buscar el siguiente nodo no visitado con menor coste
    no_visitado=setdiff(n_cercanos(:,1),n_sacados(:,1)); %Esta funcion compara que nodos han sido sacados con los que no
    candidatos=n_cercanos(ismember(n_cercanos(:,1),no_visitado),:);
    if isempty(candidatos)
        break;
    end 
    candidatos=sortrows(candidatos,3);
    index=candidatos(1,1);
end

%FASE DE RECONSTRUCCIÓN
ruta=fin;
actual=fin;
while actual~=inicio
    pred=n_cercanos(actual,4);
    if pred==0
        ruta=[];
        break;
    end
    ruta=[pred ruta];
    actual=pred;
end

%Coste final

coste=n_cercanos(fin,2);
if(isinf(coste))
    ruta=[];
end 
end 