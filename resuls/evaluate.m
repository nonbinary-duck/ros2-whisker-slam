## Copyright (C) 2024 Villanelle
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.

## -*- texinfo -*-
## @deftypefn {} {@var{retval} =} evaluate (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Villanelle <villanelle@pop-os>
## Created: 2024-05-08

function retval = evaluate ()

    # Clean up before we start
    close all;
    clear;

    # This function is expanded upon from my Image Processing assignment 1

    # Load our images
    # GT as in ground truth
    # OB as in observed
    GT = imread("ground_truth_corrected_crop.pgm");
    OB = imread("whisker.pgm");

    # Get the dimension of our image
    # They both must have the same size
    dim = size(GT);

    # Get only the obstacle layers
    obstacle_GT = (GT == zeros(dim, "uint8"));
    obstacle_OB = (OB == zeros(dim, "uint8"));

    #
    # Display our obstacle layers
    #
    figure();
    subplot(1,2,1);
        imshow(obstacle_GT);
        title("Ground truth obstacle map");
    subplot(1,2,2);
        imshow(obstacle_OB);
        title("Observed obstacle map");

    # Get data
    # For the true positive we want to return boolean data from an
    # arithmatic operation
    TP = (obstacle_OB - not(obstacle_GT)) == 1;

    # Get pixels incorectly classified as an obstacle
    FP = ((not(obstacle_GT) - obstacle_OB) - obstacle_GT) == 0;

    # Get pixels corectly classified as not an obstacle
    TN = (not(obstacle_OB) - obstacle_GT) == 1;

    # Get pixels incorectly classified as not an obstacle
    FN = (not(obstacle_GT == obstacle_OB) .* obstacle_GT) == 1;


    #
    # Display our data
    #
    figure();
    imshow(TP); title("True Positives");

    figure();
    imshow(FP); title("False Positives");

    figure();
    imshow(TN); title("True Negatives");

    figure();
    imshow(FN); title("False Negatives");

    #
    # Sanity checks
    #

    # Positives
    sanity_P      = (TP+FP) == 1;
    sanity_P_true = sanity_P == obstacle_OB;

    # Negatives
    sanity_N      = (TN+FN) == 1;
    sanity_N_true = sanity_N == not(obstacle_OB);

    # Print out if there's any error
    sanity_P_tsum = sum(sum( not(sanity_P_true) ));
    sanity_N_tsum = sum(sum( not(sanity_N_true) ));

    if (sanity_P_tsum > 0)
        printf("Bad maths for positive by %i\n", sanity_P_tsum);
    endif

    if (sanity_N_tsum > 0)
        printf("Bad maths for negative by %i\n", sanity_N_tsum);
    endif

    # Display our sanity checks
    figure();
    subplot(2,2,1);
        imshow(sanity_P);
        title("Sanity check positive");
    subplot(2,2,2);
        imshow(sanity_N);
        title("Sanity check negative");

    # Print the debug screens (make errors white so more visible)
    subplot(2,2,3);
        imshow( not(sanity_P_true) );
        title("Sanity check positive (should be all black)");
    subplot(2,2,4);
        imshow( not(sanity_N_true) );
        title("Sanity check negative (should be all black)");


    #
    # Get statistics
    #

    # Get some raw data for later
    count_gt = sum(sum(obstacle_GT));
    count_ob = sum(sum(obstacle_OB));

    # We need counts of the values
    tp = sum(sum(TP));
    fp = sum(sum(FP));
    tn = sum(sum(TN));
    fn = sum(sum(FN));

    # Compute the statistics
    # Formulas from Grandini, Bagli and Visani (2020)
    precision = (tp) / (tp + fp);
    recall    = (tp) / (tp + fn);
    accuracy  = (tp + tn) / (tp + tn + fp + fn);
    f1_score  = (2) / ( power(precision, -1.0) + power(recall, -1.0) );

    # Print to the console!
    printf("\n");
    printf("============\n");
    printf("= Metrics  =\n");
    printf("============\n\n");
    printf("Precision: %f\nRecall:    %f\nAccuracy:  %f\nF1-score:  %f\n\n", precision, recall, accuracy, f1_score);

    # Also print out some raw data
    printf("============\n");
    printf("= Raw Data =\n");
    printf("============\n\n");
    printf("Count of ground-truth cells: %i\nCount of observed cells:     %i\n", count_gt, count_ob);
    printf("\n");
    printf("True positive:  %i\nFalse positive: %i\nTrue negative:  %i\nFalse negative: %i\n", tp, fp, tn, fn);
    printf("\n");

    # Also get the sum of changes to the ground-truth
    GT_ORIG  = imread("ground_truth_crop.pgm");
    delta_gt = sum(sum(GT != GT_ORIG));
    printf("Pixel difference between ground_truth_crop and gt_corrected_crop: %i\n", delta_gt);

    printf("\n");

endfunction
