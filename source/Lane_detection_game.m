%% Video Loading
reader = vision.VideoFileReader('test_video.avi');
viewer = vision.DeployableVideoPlayer;
image = screenShotRGB(350,175,670, 475);
%image = step(reader);
[h,w,d] = size(image);

%% CONSTANTS DEFINITION
n = 3; % Square filter size
H_PEAKS = 20;
TH_PEAKS = 0.2;
ROI_correction = 60;
theta_inp = 55;
ROI_width = [1:w];
ROI_height = [1:h];
ROI_height=ROI_height((ceil(h/2)+ROI_correction):end);
first_split_iter = true;
straight = true;
UNION_ANGLE = 30 ;
UPPER_SPLIT_LIMIT = -37;
LOWER_SPLIT_LIMIT = -75;



best_r_lane = [w/2; w/3];
best_l_lane = [2*w/3; 2*w/3];
%% Video Loop
count = 1;
i=1;
while true
    
    %image = step(reader);
    image = screenShotRGB(350,175,670, 475);
    %% Preprocessing the frame
    gray_image=rgb2gray(image);   % 0.2989*R+0.5870*G+0.1140*B

    ROI=gray_image(ROI_height,ROI_width);
    [h_ROI,w_ROI,d_ROI]=size(ROI);
    denoised_ROI = colfilt(ROI,[n n],'sliding',@median);

    %th=adaptthresh(ROI);
    th = multithresh(image);
    BW_ROI =(denoised_ROI-th)*255;
    edges_ROI = edge(BW_ROI,'Sobel',[],'both');
    
    %% Processing the lane selected in the frame before
     x_inp = best_l_lane; 
     y_inp=[h-h_ROI h];
     theta_inp_l=-(180/pi)*atan((x_inp(1)-x_inp(2))/(y_inp(1)-y_inp(2)));
     rho_inp_l = x_inp(1)*cos(theta_inp*pi/180)+y_inp(1)*sin(theta_inp*pi/180);
     
    %% Deciding if we are on a straight line or on a curve
  if theta_inp_l<=UPPER_SPLIT_LIMIT && theta_inp_l>=LOWER_SPLIT_LIMIT
     % Hough Transform
     [H,theta,ro] = hough(edges_ROI);
     % Peak detection
     H1 = H;
     %Wipe out H matrix with theta < -70 deg and theta >= 70 deg
     H1(:, 1:20) = 0;
     H1(:, end-20:end) = 0;
     
     P  = houghpeaks(H1,H_PEAKS,'Threshold',TH_PEAKS*max(H1(:)));
     Lines= [ro(P(:, 1)); theta(P(:, 2))];
     
     % Formula y=(-cos(theta)/sin(theta))*x+(ro/sin(theta))
     y1=zeros(1,length(Lines(1,:)));% y=0
     x1=round(Lines(1,:)./cos(Lines(2,:).*(pi/180)));
     y2=h_ROI.*ones(1,length(Lines(1,:)));
     x2=round((sin(Lines(2,:).*(pi/180))./cos(Lines(2,:).*(pi/180))).*((Lines(1,:)./sin(Lines(2,:).*(pi/180)))-y2));
     
     x=[x1;x2]; % x values of the lines
     y1_f=(h-h_ROI)*ones(1,length(Lines(1,:)));
     y2_f=(h)*ones(1,length(Lines(1,:)));
     y=[y1_f;y2_f]; % y values of the lines
     straight = true;
     
  else % When we are in a bend we divide the ROI in two for a better approximation
     % Upper ROI
     edges_ROI_a=edges_ROI((1:floor(length(edges_ROI(:,1))/2)),:);
     [h_ROI_a,w_ROI_a,d_ROI_a]=size(edges_ROI_a);
     % Hough Transform
     [H_a,theta_a,ro_a] = hough(edges_ROI_a);
     H_a(:, 1:20) = 0;
     H_a(:, end-20:end) = 0;
     P_a  = houghpeaks(H_a,H_PEAKS,'Threshold',TH_PEAKS*max(H_a(:)));
     Lines_a= [ro_a(P_a(:, 1)); theta_a(P_a(:, 2))];

     %Lines_a= [ro_a(P_a(:, 1)); theta_a(P_a(:, 2))];  %Candidate lines
     y1_a=zeros(1,length(Lines_a(1,:)));
     x1_a=round(Lines_a(1,:)./cos(Lines_a(2,:).*(pi/180)));
     y2_a=h_ROI_a.*ones(1,length(Lines_a(1,:)));
     x2_a=round((sin(Lines_a(2,:).*(pi/180))./cos(Lines_a(2,:).*(pi/180))).*((Lines_a(1,:)./sin(Lines_a(2,:).*(pi/180)))-y2_a));
     
     x_a=[x1_a;x2_a]; % x values of the lines
     y1_fa=(h-h_ROI)*ones(1,length(Lines_a(1,:)));
     y2_fa=(h-h_ROI+h_ROI_a)*ones(1,length(Lines_a(1,:)));
     y_a=[y1_fa;y2_fa]; % y values of the lines
     
     % Lower ROI
     edges_ROI_b=edges_ROI((floor(length(edges_ROI(:,1))/2)+1):end,:);
     [h_ROI_b,w_ROI_b,d_ROI_b]=size(edges_ROI_b);
     
     [H_b,theta_b,ro_b] = hough(edges_ROI_b);
     H_b(:, 1:20) = 0;
     H_b(:, end-20:end) = 0;
     P_b  = houghpeaks(H_b,H_PEAKS,'Threshold',TH_PEAKS*max(H_b(:)));
     Lines_b= [];  %Candidate lines
     Lines_b= [ro_b(P_b(:, 1)); theta_b(P_b(:, 2))];

     %Lines_b= [ro_b(P_b(:, 1)); theta_b(P_b(:, 2))];  %Candidate lines
     y1_b=zeros(1,length(Lines_b(1,:)));
     x1_b=round(Lines_b(1,:)./cos(Lines_b(2,:).*(pi/180)));
     y2_b=h_ROI_b.*ones(1,length(Lines_b(1,:)));
     x2_b=round((sin(Lines_b(2,:).*(pi/180))./cos(Lines_b(2,:).*(pi/180))).*((Lines_b(1,:)./sin(Lines_b(2,:).*(pi/180)))-y2_b));
     
     x_b=[x1_b;x2_b]; % x values of the lines
     y1_fb=(h-h_ROI+h_ROI_b)*ones(1,length(Lines_b(1,:)));
     y2_fb=(h)*ones(1,length(Lines_b(1,:)));
     y_b=[y1_fb;y2_fb]; % y values of the lines
     straight = false;
     
  end  
    
    %% Lane tracking
  if straight
    % divide lines in 2 sets: Left lane and Right lane candidates
    x1_r_cand = x1(x1>x2)';
    x2_r_cand = x2(x2<x1)';
    x1_l_cand = x1(x1<x2)';
    x2_l_cand = x2(x2>x1)';
    
    x_r = [x1_r_cand x2_r_cand];
    x_l = [x1_l_cand x2_l_cand];
    
    best_r_lane_l = repmat(best_r_lane',length(x_r(:,1)),1);
    best_l_lane_l = repmat(best_l_lane',length(x_l(:,1)),1);
    % Choose best candidates based on the kalman filter predictions
    if ~isempty(x_r) % Make sure there are candidates
            % choose best right candidate
            z = 1:length(x_r(:,1));
            i = sum(((x_r-best_r_lane_l).^2)')' == min(sum(((x_r-best_r_lane_l).^2)')');
            j=z(i==1);
            best_r_lane = x_r(j(1),:)';
    end
    if ~isempty(x_l) % Make sure there are candidates
            % choose best left candidate
            z = 1:length(x_l(:,1));
            i = sum(((x_l-best_l_lane_l).^2)')' == min(sum(((x_l-best_l_lane_l).^2)')');
            j=z(i==1);
            best_l_lane = x_l(j(1),:)';
    end
    
    x = [best_r_lane best_l_lane]';
    y = y(1:2,1:2)'; 

    p_lines = [x(:,1) y(:,1) x(:,2) y(:,2)];
    p_poly = [p_lines(1,1:2) p_lines(1,3:4) p_lines(2,3:4) p_lines(2,1:2)];
    first_split_iter = true;
  else
    % upper ROI
    % divide lines in 2 sets: Left lane and Right lane candidates
    x1_a_r_cand = x1_a(x1_a>x2_a)';
    x2_a_r_cand = x2_a(x2_a<x1_a)';
    x1_a_l_cand = x1_a(x1_a<x2_a)';
    x2_a_l_cand = x2_a(x2_a>x1_a)';
    % lower ROI
    % divide lines in 2 sets: Left lane and Right lane candidates
    x1_b_r_cand = x1_b(x1_b>x2_b)';
    x2_b_r_cand = x2_b(x2_b<x1_b)';
    x1_b_l_cand = x1_b(x1_b<x2_b)';
    x2_b_l_cand = x2_b(x2_b>x1_b)';
    
    x_r_top = [x1_a_r_cand x2_a_r_cand];
    x_l_top = [x1_a_l_cand x2_a_l_cand];
    x_r_bottom = [x1_b_r_cand x2_b_r_cand];
    x_l_bottom = [x1_b_l_cand x2_b_l_cand];
    
    % Define previous best lines for the two regions
    if first_split_iter
       best_l_lane_top = [best_l_lane(1,1);(best_l_lane(1,1)+best_l_lane(2,1))/2];
       best_r_lane_top = [best_r_lane(1,1);(best_r_lane(1,1)+best_r_lane(2,1))/2];
       best_l_lane_bottom = [(best_l_lane(1,1)+best_l_lane(2,1))/2;best_l_lane(2,1)];
       best_r_lane_bottom = [(best_r_lane(1,1)+best_r_lane(2,1))/2;best_r_lane(2,1)];
       first_split_iter = false;
    end
    
    best_r_lane_top_l = repmat(best_r_lane_top',length(x_r_top(:,1)),1);
    best_l_lane_top_l = repmat(best_l_lane_top',length(x_l_top(:,1)),1);
    best_r_lane_bottom_l = repmat(best_r_lane_bottom',length(x_r_bottom(:,1)),1);
    best_l_lane_bottom_l = repmat(best_l_lane_bottom',length(x_l_bottom(:,1)),1);
    
    % Upper ROI
    if ~isempty(x_r_top) % Make sure there are candidates
            % choose best right top candidate
            z = 1:length(x_r_top(:,1));
            i = sum(((x_r_top-best_r_lane_top_l).^2)')' == min(sum(((x_r_top-best_r_lane_top_l).^2)')');
            j=z(i==1);
            best_r_lane_top = x_r_top(j(1),:)';
    end
    if ~isempty(x_l_top) % Make sure there are candidates
            % choose best left candidate
            z = 1:length(x_l_top(:,1));
            i = sum(((x_l_top-best_l_lane_top_l).^2)')' == min(sum(((x_l_top-best_l_lane_top_l).^2)')');
            j=z(i==1);
            best_l_lane_top = x_l_top(j(1),:)';
    end
    % Lower ROI
    if ~isempty(x_r_bottom) % Make sure there are candidates
            % choose best right candidate
            z = 1:length(x_r_bottom(:,1));
            i = sum(((x_r_bottom-best_r_lane_bottom_l).^2)')' == min(sum(((x_r_bottom-best_r_lane_bottom_l).^2)')');
            j=z(i==1);
            best_r_lane_bottom = x_r_bottom(j(1),:)';
    end
    if ~isempty(x_l_bottom) % Make sure there are candidates
            % choose best left candidate
            z = 1:length(x_l_bottom(:,1));
            i = sum(((x_l_bottom-best_l_lane_bottom_l).^2)')' == min(sum(((x_l_bottom-best_l_lane_bottom_l).^2)')');
            j=z(i==1);
            best_l_lane_bottom = x_l_bottom(j(1),:)';
    end
    
    x = [best_r_lane_top best_l_lane_top best_r_lane_bottom best_l_lane_bottom];
    y = [y_a(1:2,1:2) y_b(1:2,1:2)]; 
    
    p_lines = [x(1,:)' y(1,:)' x(2,:)' y(2,:)'];

    p_poly = [p_lines(1,1:2) p_lines(1,3:4) p_lines(2,3:4) p_lines(2,1:2)];
    p_poly = [ p_poly; p_lines(3,1:2) p_lines(3,3:4) p_lines(4,3:4) p_lines(4,1:2)];
    
    first_split_iter = false;
    
    theta_lane_a=-(180/pi)*atan((best_l_lane_top(1)-best_l_lane_top(2))/(y(1,1)-y(2,1)));
    theta_lane_b=-(180/pi)*atan((best_l_lane_bottom(1)-best_l_lane_bottom(2))/(y(1,3)-y(2,3)));
    
    if abs(theta_lane_a-theta_lane_b)<=UNION_ANGLE
        straight = true;
        best_l_lane = [best_l_lane_top(1); best_l_lane_bottom(2)];
        best_r_lane = [best_r_lane_top(1); best_r_lane_bottom(2)];
        y = [h-h_ROI; h];
    end
    
  end
    %% Plot lines and polygon in the video
    image_with_poly = insertShape(image, 'FilledPolygon', p_poly ,'Color','red','Opacity',0.2);
    image_with_lines = insertShape(image_with_poly,'Line',p_lines,'LineWidth',2,'Color','green');
    step(viewer, image_with_lines)
    i=i+1;
end