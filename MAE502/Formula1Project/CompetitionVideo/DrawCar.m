% DrawCar: draw an individual car (as bicycle model only)
function L=DrawCar(Car,x,y,psi,gamma,color)
wheelscale=2;
pfront=Car.a*[cos(psi);sin(psi)]+[x;y];
prear=-Car.b*[cos(psi);sin(psi)]+[x;y];
Xcar=[x;y];
tv=[cos(psi);sin(psi)];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];
% front wheel angle
Rotate=[cos(psi+gamma) -sin(psi+gamma)
        sin(psi+gamma)  cos(psi+gamma) ];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0],pfront+Rotate*[wheelscale*Car.Rw;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
subplot(3,2,[1 3]);
L(1)=plot(Xcar(1),Xcar(2),'o','color',color,'MarkerSize',10);
L(2)=plot(vline(1,:),vline(2,:),'color',color,'LineWidth',3);
L(3)=plot(frontwheelline(1,:),frontwheelline(2,:),'color',color,'LineWidth',5);
L(4)=plot(rearwheelline(1,:),rearwheelline(2,:),'color',color,'LineWidth',5);
drawnow;
subplot(3,2,[2 4]);
L(5)=plot(Xcar(1),Xcar(2),'o','color',color,'MarkerSize',10);
drawnow;
end