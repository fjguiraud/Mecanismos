function grashof = grashof(a, b,c,d)
  v1 = [a,b,c,d];
   disp(" ")
  disp ("Manivela | Acoplador | Seguidor | Tierra");
  disp(v1);
  
  disp("Ordenando el vector");
  v2 = sort(v1);
  disp(v2);
  disp("Long. del eslab. mas corto (S): "),disp(v2(1,1));
  s=v2(1,1);
  disp("Long. del eslab. mas largo (L): "),disp(v2(1,4));
  l=v2(1,4);
  disp("Long. del eslab. restante (P): "),disp(v2(1,3));
  p=v2(1,3);
  disp("Long. del eslab. restante (Q): "),disp(v2(1,2));
  q=v2(1,2);
  
  disp(" ")
  
  
  if l>s+p+q 
  disp("L > S + P + Q") 
  disp("No es posible unir eslabones")
  
elseif l == (s+p+q) 
  disp("L = S + P + Q") 
  disp("Es una estructura")
  
  else
  
   if s+l<p+q 
     disp("L + S < P + Q") 
   
   disp("Clase I")
   disp("Es capaz de realizar rotacion completa")
    if s==v2(1,2)
    disp ("-Mecanismo doble seguidor de Grashof") 
    elseif s==v2(1,1) || s==v2(1,3)
    disp("-Mecanismo manivela seguidor")
    else 
    disp ("-Mecanismo doble manivela") 
    endif
   
 elseif s+l>p+q
   disp("L + S > P + Q") 
   disp("Clase II")
   disp("Ningun eslabon puede rotar completamente") 
   disp("Todas las inversiones seran balancines triples")
   disp("-Mecanismo triple seguidor")
 elseif (s+l) == (p+q)
   disp("L + S = P + Q") 
   disp("Clase III")
   disp("Caso especial de Grashof")
   disp("Todas las inversiones seran bielas")
   endif
  endif
  
endfunction