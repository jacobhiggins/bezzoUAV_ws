% save P R

dlmwrite('reward.txt', R, 'delimiter', ' ');
dlmwrite('transition.txt', P(:,:,1), 'delimiter', ' ');
dlmwrite('transition.txt', P(:,:,2), '-append', 'delimiter', ' ');
dlmwrite('transition.txt', P(:,:,3), '-append', 'delimiter', ' ');
dlmwrite('transition.txt', P(:,:,4), '-append', 'delimiter', ' ');
