function F_s = SocialForce(person, i, s_new)
r = 0.5; %radius around each person =0.5 m
a = 1; %magnitude of force
b = 1; %range of force
lambda = 0.5; % scaling factor
current_dir = s_new - person{i}.s;
F_social = [0 0];
for j=1:length(person)
    if j~=i
        n_ij = (person{i}.s - person{j}.s)/norm((person{i}.s - person{j}.s));% normalized vector pointing j to i
        cos_phi = (-n_ij(1)*current_dir(1)-n_ij(2)*current_dir(2))/(norm(n_ij)*norm(current_dir));
        d_ij = norm(person{i}.s - person{j}.s);
        F_social = F_social + a*exp((r-d_ij)/b)*n_ij*(lambda+(1-lambda)*(1+cos_phi)/(2));
    end
end
F_s = current_dir + F_social;
end