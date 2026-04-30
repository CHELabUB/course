% DrawCar: draw an individual car (as bicycle model only)
function L=DrawCarSimple(Car,x,y,psi,gamma,color)
Xcar=[x;y];
subplot(3,2,[1 3]);
L(1)=plot(Xcar(1),Xcar(2),'o','color',color,'MarkerSize',10);
drawnow;
subplot(3,2,[2 4]);
L(2)=plot(Xcar(1),Xcar(2),'o','color',color,'MarkerSize',10);
drawnow;
end