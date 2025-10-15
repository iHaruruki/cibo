各点を3次元座標ベクトルとして (\mathbf{p}{\mathrm{nose}},\ \mathbf{p}{\mathrm{hand_R}},\ \mathbf{p}{\mathrm{hand_L}},\ \mathbf{p}{\mathrm{chin}},\ \mathbf{p}{\mathrm{upper_lip}},\ \mathbf{p}{\mathrm{lower_lip}}\in\mathbb{R}^3) とする。ユークリッド距離（L2ノルム）を用いると，TeX形式は次のようになる。

$$ d_{h,R}=|\mathbf{p}{\mathrm{nose}}-\mathbf{p}{\mathrm{hand_R}}|2 =\sqrt{(x{\mathrm{nose}}-x_{\mathrm{hand_R}})^2+(y_{\mathrm{nose}}-y_{\mathrm{hand_R}})^2+(z_{\mathrm{nose}}-z_{\mathrm{hand_R}})^2} $$

$$ d_{h,L}=|\mathbf{p}{\mathrm{nose}}-\mathbf{p}{\mathrm{hand_L}}|2 =\sqrt{(x{\mathrm{nose}}-x_{\mathrm{hand_L}})^2+(y_{\mathrm{nose}}-y_{\mathrm{hand_L}})^2+(z_{\mathrm{nose}}-z_{\mathrm{hand_L}})^2} $$

$$ d_j=|\mathbf{p}{\mathrm{nose}}-\mathbf{p}{\mathrm{chin}}|2 =\sqrt{(x{\mathrm{nose}}-x_{\mathrm{chin}})^2+(y_{\mathrm{nose}}-y_{\mathrm{chin}})^2+(z_{\mathrm{nose}}-z_{\mathrm{chin}})^2} $$

$$ d_m=|\mathbf{p}{\mathrm{upper_lip}}-\mathbf{p}{\mathrm{lower_lip}}|2 =\sqrt{(x{\mathrm{upper}}-x_{\mathrm{lower}})^2+(y_{\mathrm{upper}}-y_{\mathrm{lower}})^2+(z_{\mathrm{upper}}-z_{\mathrm{lower}})^2} $$