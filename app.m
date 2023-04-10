%% `matlab-trimesh-stereo-reconstruction`
%> ========================================================================
%>
%>  @file matlab-trimesh-stereo-reconstruction/script.m
%>  @brief Tesellating stereo reconstruction script via point cloud processing.
%>  @requires $matlabroot/toolbox/vision
%>  @requires surf2stl
%>
%> ========================================================================
%>
%> MATLAB Computer Vision Toolbox script for stereo reconstruction as a
%> tesselating 3D triangular mesh surface in STL format.
%>
%> This script is unique, because it implements userland logic for:
%> - normalising a binocular disparity data map by combining multiple
%>   match methods
%> - model-filtering the scene limits of the generated point cloud
%> - post-processing the scene point cloud with an interpolating signal
%>   filter
%> - normalising the Cartesian (X-Y-Z) output axes in the resulting STL
%>
%> **File tree:**
%> - `./script.m`: a command window script
%> - `./app.m`: a GUI script
%> - `./live.mlx`: a live script
%> - Image folders - `./data/*`:
%>   - `/input`: actual 3D modelling target scene
%>   - `/config/left`: left stereo view with checkerboard
%>   - `/config/right`: right stereo view with checkerboard
%> - Documentation - `./assets`
%> - Output - `./point-cloud.stl` (checked out)
%>
%> ========================================================================
%% Configuration
%> ========================================================================
%>
%% @subsection Setup
%>
%> **Input:** collect grayscale stereo photos in landscape using the
%> `*.jpg` / `*.jpeg` format (can use any extension from
%> [`imformats`](/help/matlab/ref/imformats.html).
%>  - Add checkerboard images from left stereo view to `./config/left`
%>  - Add checkerboard images from right stereo view to `./config/right`
%>  - Add modelling input images from stereo view to `./input`
%>
%>     imformats() %  supported images formats in MATLAB
%>
%> **Output:** A 3D representation of the stereo image set in STL format.
%>         The default location is `./point-cloud.stl`.
%>
%> **N.B:** For good performance:
%>  - resize your images below 720p (maybe between 360p and 480p)
%>  - use [GIMP](https://www.gimp.org/) and [BIMP](https://alessandrofrancesconi.it/projects/bimp/) to convert the image color space to grayscale
%>
%% @subsection Image support
%>
%> - Must be the same orientation as the checkerboard to reduce pixel
%>    error from reprojection.
%> - For calibration images:
%>   - asymmetric (odd-even) checkerboard should be in all views
%>   - minimum image count per folder is 4 (for low reprojection error)
%>   - naming convention: `./config/<VIEW>/<VIEW>##.jpg`
%>     (e.g. `./config/left/left01.jpg`)
%>
%> ```matlab
%> imformats() % supported images formats in MATLAB
%> ```
%>
%> **N.B:** the image file names must be numbered in ascending order.

%> ========================================================================
%% Source code
%> ========================================================================

%> ========================================================================
%% @section Setup

%> ========================================================================
%% @subsection Environment cleanup
close all;
clear;
clc;

%> ========================================================================
%% @subsection Show figure modal
%>  @figure Checkerboard boundary points for calibration experiment.
%>  @figure Reprojection errors for calibration experiment.
%>  @figure Stereo anaglyph image of input scene.
%>  @figure Disparity map as `parula` colormap image.
%>  @figure 3D connected surface mesh plot of the point cloud.
figure2D = figure;
figure2D.Units = 'normalized';
movegui(figure2D, 'center');
figure2D.OuterPosition = [0 0 1 1];
rotate3d on;

%> ========================================================================
%% @subsection Workspace cleanup
filePath = fullfile(pwd, 'data', 'toolbox');
stlPath = 'point-cloud.stl';
if exist(fullfile(filePath, stlPath), 'file')
    recycle on;
    delete(fullfile(filePath, stlPath));
end

%> ========================================================================
%% @subsection Input loading
inputImages = imageDatastore(fullfile(filePath, 'input'));
I1 = readimage(inputImages, 1);
if size(I1, 3) == 3
    I1 = rgb2gray(I1);
end
I2 = readimage(inputImages, 2);
if size(I2, 3) == 3
    I2 = rgb2gray(I2);
end

%> ========================================================================
%% @subsection Dependency management

if ~exist('surf2stl', 'file')
    if ~matlab.addons.isAddonEnabled('mpm')
        error([                                                         ...
            'Please install MPM as a MATLAB Addon.\n'                   ...
            '<a href="'                                                 ...
            '/matlabcentral/fileexchange/54548' ...
            '">'                                                        ...
            'mpm - File Exchange - MATLAB Central'                      ...
            '</a>'                                                      ...
        ])
    else
        mpm install surf2stl;
    end
end

%> ========================================================================
%% @subsection User configuration
%>  @var filePath path to current folder.
%>  @var stlPath name of point cloud STL.
%>  @var imageMinimum min. no of images in calib folder.
%>  @var squareWidth Checkerboard square width in mm.
%>  @var ptCloudDensity Point density within squareWidth.
%>  @var sGolayFiltOrder Savitsky-Golay extrapolation curve order.
%>  @var sGolayFiltFrameLen Savitsky-Golay sliding window point count.
%>  @var disparityBMBias Algorithm bias to block matching vs semi
%>       global matching (in the range -1 to 1).
%>  @var disparityMaxRatio Ratio for disparity map ceiling vs maximum
%>       disparity (in the range 0 to 1).
squareWidth = 50;
ptCloudDensity = 5;
sGolayFiltOrder = 3;
sGolayFiltFrameLen = 21;
disparityBMBias = -0.5;
disparityMaxRatio = 0.675;

dlgtitle = 'Stereo reconstruction of STL tesselating mesh';

prompt = string({});
definput = string({});
prompt{1} = 'Checkerboard square width (mm)';
definput{1} = num2str(squareWidth);
prompt{2} = 'Point cloud density - ratio vs square width';
definput{2} = num2str(ptCloudDensity);
prompt{3} = 'Savitsky-Golay extrapolation curve order';
definput{3} = num2str(sGolayFiltOrder);
prompt{4} = 'Savitsky-Golay sliding window point count';
definput{4} = num2str(sGolayFiltFrameLen);
prompt{5} = 'Algorithm bias to block matching vs semi-global matching (in the range -1 to 1).';
definput{5} = num2str(disparityBMBias);
prompt{6} = 'Ratio for disparity map ceiling vs maximum disparity (in the range 0 to 1).';
definput{6} = num2str(disparityMaxRatio);

answer = inputdlg(prompt, dlgtitle, 1, definput);

if ~isnan(answer{1}); squareWidth = str2double(answer{1}); end
if ~isnan(answer{2}); ptCloudDensity = str2double(answer{2}); end
if ~isnan(answer{3}); sGolayFiltOrder = str2double(answer{3}); end
if ~isnan(answer{4}); sGolayFiltFrameLen = str2double(answer{4}); end
if ~isnan(answer{5}); disparityBMBias = str2double(answer{5}); end
if ~isnan(answer{6}); disparityMaxRatio = str2double(answer{6}); end

%> ========================================================================
%% @subsection Show loading indicator
%> Script takes $T = ~30s$ to execute.
wb = waitbar(0, 'Loading');
wb.Visible = 'on';

%> ========================================================================
%% @section Camera calibration of stereo images
%> Code based on MATLAB `rectifyStereoImages` code sample [1].

%> ========================================================================
%% @subsection Image loading
%>  Using a grayscale color space reduces image data & overhead in
%>  calibration phase. [2]
%>
%>  @var inputImages Images from `./input` subfolder.
%>  @var calibLeftImages Images from `./config/left` subfolder.
%>  @var calibRightImages Images from `./config/right` subfolder.
%>  @see #image-support
waitbar(0, wb, 'Loading input images.');
calibLeftImages = imageDatastore(fullfile(filePath, 'config', 'left'));
calibRightImages = imageDatastore(fullfile(filePath, 'config', 'right'));

%> ========================================================================
%% @subsection Image validation
%>  @exception char mismatch in image count between `./config/left` & `./config/right`.
%>  @exception char below 4 images in `./config/left` & `./config/right`.
%>  @exception char mismatch in resolution of `./input` images.
S1 = [size(I1, 1), size(I1, 2)];
S2 = [size(I2, 1), size(I2, 2)];

imageAmounts = struct;
imageAmounts.L = size(calibLeftImages.Files, 1);
imageAmounts.R = size(calibRightImages.Files, 1);

errno = cell(3);
errno{1} = 'stereo2trimesh::ERR_MISMATCH_IMG_COUNT';
errno{2} = 'stereo2trimesh::ERR_CALIB_IMG_INSUFFICIENT';
errno{3} = 'stereo2trimesh::ERR_MISMATCH_IMG_DIM';

if imageAmounts.L ~= imageAmounts.R
    e = [errno{1} ' (L: ' imageAmounts.L ', R: ' imageAmounts.R];
    errordlg(e);
    error(e);
elseif imageAmounts.L < 4
    e = [errno{2} ' (n=' imageAmounts.L ')'];
    errordlg(e);
    error(e);
elseif ~isequal(S1, S2)
    e = [errno{3} ' (L: ' S1(1) 'x' S1(2) 'px, R: ' S2(1) 'x' S2(2) 'px)'];
    errordlg(e);
    error(e);
end

%> ========================================================================
%% @subsection Checkerboard detection
waitbar(0.1, wb, 'Detecting checkerboard keypoints.');
[imagePoints, boardSize] = detectCheckerboardPoints(                    ...
    calibLeftImages.Files,                                              ...
    calibRightImages.Files                                              ...
);

%> ========================================================================
%% @subsection Calculate checkerboard keypoints
worldPoints = generateCheckerboardPoints(boardSize, squareWidth);

%> ========================================================================
%% @subsection Calibrate stereo camera system
%>  @param EstimateSkew Are image axes exactly perpendicular?
%>         Default: `true`.
%>  @param EstimateTangentialDistortion Factor in whether the camera is
%>         horizontal. Default: `true`.
%>  @param NumRadialDistortionCoefficients Good for fish-eye lenses.
%>         Default: `2`.
%>  @param ImageSize Matrix for size of image - `imageSize`.
%>  @todo  Adjust `estimateCameraParameters` parameters for experimental
%>         stage.
waitbar(0.2, wb, "Estimating camera parameters.");
[stereoParams, ~, estimationErrors] = estimateCameraParameters(         ...
    imagePoints, worldPoints,                                           ...
    'EstimateSkew', true,                                               ...
    'EstimateTangentialDistortion', false                               ...
);

%> ========================================================================
%% @subsection Display camera extrinisics
%>  Reprojection is the process of "reprojecting" original image from a
%>  camera image. Most camera images have distortion (e.g. "fisheye" lens
%>  effect).
waitbar(0.3, wb, "Showing camera extrinisics.");
A1 = subplot(2, 2, 1);
showExtrinsics(stereoParams, "CameraCentric");
view([-45 45]);

%> ========================================================================
%% @subsection Graph camera reprojection errors
waitbar(0.4, wb, "Showing reprojection errors.");
A2 = subplot(2, 2, 2);
showReprojectionErrors(stereoParams);
displayErrors(estimationErrors, stereoParams);

%> ========================================================================
%% @subsection Stereo rectification with "valid" output view
%>  The "valid" option is most suitable for computing disparity. 
%>  These images have negative polar distortion and appear concave
%>  (the top has a U-curve). It limits the rectified image data from
%>  to a regular 2D rectangle. [3]
%>
%>  @param OutputView Crop the image to a rectangle, fitting inside the
%>         overlapping, curved 3D anaglyph. Default: `valid`.
waitbar(0.5, wb, "Showing stereo rectification.");
[F1, F2] = rectifyStereoImages(I1, I2, stereoParams, 'OutputView', 'valid');
pixelDensityMm = mrdivide(                                              ...
    mean([                                                              ...
        stereoParams.CameraParameters1.FocalLength,                     ...
        stereoParams.CameraParameters2.FocalLength                      ...
    ], 2),                                                              ...
    mean([                                                              ...
        stereoParams.CameraParameters1.IntrinsicMatrix(1, 1),           ...
        stereoParams.CameraParameters2.IntrinsicMatrix(1, 1)            ...
    ], 2)                                                               ...
);
approxImageHeight = 2 * mean([size(F1, 1), size(F2, 1)], 2) / pixelDensityMm;
approxImageWidth = 2 * sqrt(2) * mean([size(F1, 2), size(F2, 2)], 2) / pixelDensityMm;

%> ========================================================================
%% @subsection Display an "valid" output anaglyph image
%>  Display an anaglyph image for "valid" output view.
waitbar(0.6, wb, "Showing stereo anaglyph.");
A3 = subplot(2, 2, 3);
hold on;
labels = cell(3);
labels{1} = plot(nan, nan, 'color', 'red');
labels{2} = plot(nan, nan, 'color', 'black');
labels{3} = plot(nan, nan, 'color', 'cyan');
legend([labels{:}], {'left', '', 'right'});
imshow(stereoAnaglyph(F1, F2));
axis tight;
title 'Rectified Image';
clearvars labels;

%> ========================================================================
%% @section Disparity computation from stereo images.
%>  Code based on MATLAB `disparitySGM` code sample. [4]

%> ========================================================================
%% @subsection Compute disparity map from stereo images
%>  Generate a disparity (Cartesian *z*-depth) colormap of the scene. We
%>  take a biased average of the disparity map produced by semi-global and
%>  block matching algorithms. This ensures reduced "hole" (neutral)
%>  amplitudes in the disparity data.
%>
%>  @todo Adjust `disparityBMBias` as appropriate for the image input.
%>  @todo Adjust the range maximum to $c\times2^4,c\in\mathbb{N}$ to
%>        remove outliers or camera noise.
waitbar(0.7, wb, "Computing disparity map.");
disparityMapBM = disparityBM(F1, F2, "DisparityRange", [0, 64]);
disparityMapSGM = disparitySGM(F1, F2, "DisparityRange", [0, 64]);

disparityMapBM(isnan(disparityMapBM)) = 0;
disparityMapSGM(isnan(disparityMapSGM)) = 0;

disparityBMQuotient = (1 + disparityBMBias) / 2;
disparitySGMQuotient = (1 - disparityBMBias) / 2;

disparityMap = disparityBMQuotient * disparityMapBM + disparitySGMQuotient * disparityMapSGM;
disparityMap(disparityMap==0) = NaN;

%> ========================================================================
%% @subsection Remove "spike" transients
%>  Limit disparity values to ~90% of the maximum to remove maximal AKA
%>  "spike" transients.
disparityMapCeil = disparityMaxRatio * max(max(disparityMap));
disparityMap(disparityMap>=disparityMapCeil) = NaN;

%> ========================================================================
%% @section Display disparity map
waitbar(0.8, wb, "Showing parula colormap.");
A4 = subplot(2, 2, 4);
imshow(disparityMap, [0, 64]);
title 'Disparity Map';
axis tight;
colormap(A4, parula);
colorbar(A4, 'southoutside');

%> ========================================================================
%% @section Point cloud generation using depth data

%> ========================================================================
%% @subsection Reconstruct organised point cache matrix
%>  Produces Cartesian scatter point cloud data in m - standard STL
%>  dimensions.
waitbar(0.9, wb, "Generating point cloud.");
rawPoints3D = reconstructScene(disparityMap, stereoParams);
rawPoints3D(isinf(rawPoints3D)) = NaN;
rawPoints3D = double(rawPoints3D) ./ 1000;

%> ========================================================================
%% @subsection Initialise axial, co-ordinate point cloud cache
pointsCache = struct;
axesKeys = ["X", "Y", "Z"];
for m = 1:3
    k = char(axesKeys(m));
    p = rawPoints3D(:, :, m);
    pointsCache.(k) = p;
end
clearvars p k;

%> ========================================================================
%% @subsection Compute checkerboard centroid as struct
%>  Compute checkerboard position as a Cartesian coordinate in the point
%>  cloud. It's the mean of the co-ordinate set closest to the origin in
%>  the *z*-axis.
%
%>  @todo See if I need to change `min` in some way (assumes convex).
checkerboardCentroid = struct;
checkerboardCentroid.Z = min(min(pointsCache.Z));
checkerboardIndex = sort(find(checkerboardCentroid.Z == pointsCache.Z));
checkerboardCentroid.X = 0;
checkerboardCentroid.Y = 0;

%> ========================================================================
%% @subsection Restrict point cloud to image scene dimensions
%> Limits:
%>  - point cloud width *x* = scene image width
%>  - point cloud height *y* = sqrt(0.5) × scene image height
%>  - point cloud depth *z* = sqrt(0.5) × scene image (height + width)
waitbar(0.9, wb, "Filter-processing point cloud co-ordinates.");
limits = struct;
cacheAxes = char(fieldnames(pointsCache));

for m = 1:3
    switch m
        case 1
            bound = approxImageWidth;
        case 2
            bound = sqrt(0.5) * approxImageHeight;
        otherwise
            bound = mean([approxImageHeight, approxImageWidth], 2) / 2;
    end

    k = cacheAxes(m);
    c = checkerboardCentroid.(k);
    l = bound/1000;

    lim = [c - l, c + l];
    limits.(k) = lim;

    p = pointsCache.(k);
    p(p < lim(1) | p > lim(2)) = NaN;
    pointsCache.(k) = p;
end

clearvars k lim p;

%> ========================================================================
%% @subsection Filter point cloud for invalid values
%> Remove invalid (NaN) values inside point cloud.
%> - Values that are `+Inf` / `-Inf` / `NaN`.
%> - Points that fall outside range of point cloud.
nanPoints = ( 0                                                         ...
    | isnan(pointsCache.X)                                              ...
    | isnan(pointsCache.Y)                                              ...
    | isnan(pointsCache.Z)                                              ...
);

for m = 1:3
    k = cacheAxes(m);
    p = pointsCache.(k);
    p(nanPoints) = checkerboardCentroid.(k);
    pointsCache.(k) = p;
end

clearvars k p;

%> ========================================================================
%% @section Surface mesh conversion

%> ========================================================================
%% @subsection Surface mesh denoising and interpolation
%   Generate a organised point cloud as a struct with 1 axis per field.
%>  Code adapted from StackOverflow. [5]
%>
%>  1. The `scatteredInterpolant` factory creates interpolant. [6]
%>  2. MATLAB maps `meshgrid` regular matrix of x-y points.
%>  3. Savitzky-Golay filter used to denoise points in Z axis.
%>
%>  @see https://commons.wikimedia.org/wiki/File:Lissage_sg3_anim.gif
waitbar(0.9, wb, "Interpolating point cloud as surface mesh.");
gs = (1 / ptCloudDensity) * (squareWidth / 1000);

I = scatteredInterpolant(pointsCache.X(:), pointsCache.Y(:), pointsCache.Z(:), "natural");

gridPoints = struct;
intX = min(pointsCache.X(:)):gs:max(pointsCache.X(:));
intY = min(pointsCache.Y(:)):gs:max(pointsCache.Y(:));
[gridPoints.X, gridPoints.Y] = meshgrid(intX, intY);

gridPoints.Z = I(gridPoints.X, gridPoints.Y);
intZ1 = sgolayfilt(gridPoints.Z.', sGolayFiltOrder, sGolayFiltFrameLen);
intZ2 = sgolayfilt(gridPoints.Z, sGolayFiltOrder, sGolayFiltFrameLen);
gridPoints.Z = (intZ1.' + intZ2)/2;

%> ========================================================================
%% @subsection Restore correct Cartesian axes in coordinate system
%>  Apply geometric transforms to gridded point clouds:
%>  - $-1$ scalar transformation of *y*-axis (vertical axis of image plane)
%>  - $-1$ scalar transformation of *z*-axis (depth axis of image plane)
gridPoints = struct('X', gridPoints.X, 'Y', gridPoints.Z, 'Z', -1 .* gridPoints.Y);

%> ========================================================================
%% @subsection Create point cloud scatter matrix
points3D = double.empty();
for m = 1:3
    points3D(:, :, m) = gridPoints.(cacheAxes(m));
end

clearvars cacheAxes;

%> ========================================================================
%% @subsection Mesh triangulation view of point cloud
%>  @note We reverse the geometric transforms.
waitbar(0.925, wb, "Show surface mesh plot.");
figure;
mesh(-1 .* gridPoints.X, gridPoints.Z, -1 .* gridPoints.Y);
title '\color{black} Mesh Triangulation';
xlabel 'x (horizontal displacement in m)';
ylabel 'y (vertical displacement in m)';
zlabel 'z (scene depth in m)';

set(gcf, "Color", "w");
set(gca, "XColor", "k");
set(gca, "YColor", "k");
set(gca, "ZColor", "k");
set(gca, "LineWidth", 1);

axis equal;
view([180 -90]);

colormap gray;
colorbar southoutside;
rotate3d on;

%> ========================================================================
%% @section STL file generation from point cloud
%>  Using `surf2stl` for high stability & speed (low interpolation).
waitbar(0.95, wb, "Writing STL output..");
stlPath = char(fullfile(pwd, stlPath));
surf2stl(stlPath, gridPoints.X, gridPoints.Y, gridPoints.Z);

%> ========================================================================
%% @section Teardown
waitbar(1, wb, "Done!");
winopen(stlPath);
close(wb);

%> ========================================================================
%% References
%> @section references
%>  1. /help/matlab/ref/rgb2gray.html
%>  2. /help/vision/examples/depth-estimation-from-stereo-video.html
%>  3. /help/vision/ref/rectifystereoimages.html
%>  4. /help/vision/ref/disparitysgm.html
%>  5. https://stackoverflow.com/a/39576639
%>  6. /help/matlab/ref/scatteredinterpolant.html
