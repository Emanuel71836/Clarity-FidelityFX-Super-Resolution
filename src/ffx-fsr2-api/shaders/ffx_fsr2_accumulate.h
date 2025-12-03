// This file is part of the FidelityFX SDK.
//
// Copyright (c) 2022-2023 Advanced Micro Devices, Inc. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef FFX_FSR2_ACCUMULATE_H
#define FFX_FSR2_ACCUMULATE_H

FfxFloat32 GetPxHrVelocity(FfxFloat32x2 fMotionVector)
{
    return length(fMotionVector * DisplaySize());
}

// [FSR Clarity] Motion Vector Filter
// Samples a 3x3 neighborhood to smooth out jittery motion vectors.
// Only applies filtering if variance is detected to minimize performance impact.
FfxFloat32x2 GetFilteredMotionVector(FfxInt32x2 iPxHrPos, FfxFloat32x2 fHrUv) {
    FfxFloat32x2 centerMotion = GetMotionVector(iPxHrPos, fHrUv);
    
    // Simple variance check: compare center with immediate neighbors (cross pattern)
    // If difference is too high, trigger filtering.
    FfxFloat32x2 mLeft = GetMotionVector(iPxHrPos + FfxInt32x2(-1, 0), fHrUv);
    FfxFloat32x2 mUp   = GetMotionVector(iPxHrPos + FfxInt32x2(0, -1), fHrUv);
    
    FfxFloat32 diff = length(centerMotion - mLeft) + length(centerMotion - mUp);
    
    if (diff > 1e-4f) { // Threshold for "jitter"
        // Apply 3x3 Median Filter
        // Median is better than average for removing outliers without blurring.
        FfxFloat32 valsX[9];
        FfxFloat32 valsY[9];
        int count = 0;
        
        const FfxInt32 RADIUS = 1;
        for (FfxInt32 y = -RADIUS; y <= RADIUS; y++) {
            for (FfxInt32 x = -RADIUS; x <= RADIUS; x++) {
                FfxFloat32x2 mv = GetMotionVector(iPxHrPos + FfxInt32x2(x, y), fHrUv);
                valsX[count] = mv.x;
                valsY[count] = mv.y;
                count++;
            }
        }
        
        // Sort X
        for (int i = 0; i < 9; i++) {
            for (int j = i + 1; j < 9; j++) {
                if (valsX[i] > valsX[j]) {
                    FfxFloat32 t = valsX[i]; valsX[i] = valsX[j]; valsX[j] = t;
                }
            }
        }
        
        // Sort Y
        for (int i = 0; i < 9; i++) {
            for (int j = i + 1; j < 9; j++) {
                if (valsY[i] > valsY[j]) {
                    FfxFloat32 t = valsY[i]; valsY[i] = valsY[j]; valsY[j] = t;
                }
            }
        }
        
        return FfxFloat32x2(valsX[4], valsY[4]);
    }
    
    return centerMotion;
}
#if FFX_HALF
FFX_MIN16_F GetPxHrVelocity(FFX_MIN16_F2 fMotionVector)
{
    return length(fMotionVector * FFX_MIN16_F2(DisplaySize()));
}
#endif

void Accumulate(const AccumulationPassCommonParams params, FFX_PARAMETER_INOUT FfxFloat32x3 fHistoryColor, FfxFloat32x3 fAccumulation, FFX_PARAMETER_IN FfxFloat32x4 fUpsampledColorAndWeight)
{
    // Aviod invalid values when accumulation and upsampled weight is 0
    fAccumulation = ffxMax(FSR2_EPSILON.xxx, fAccumulation + fUpsampledColorAndWeight.www);

#if FFX_FSR2_OPTION_HDR_COLOR_INPUT
    //YCoCg -> RGB -> Tonemap -> YCoCg (Use RGB tonemapper to avoid color desaturation)
    fUpsampledColorAndWeight.xyz = RGBToYCoCg(Tonemap(YCoCgToRGB(fUpsampledColorAndWeight.xyz)));
    fHistoryColor = RGBToYCoCg(Tonemap(YCoCgToRGB(fHistoryColor)));
#endif

    const FfxFloat32x3 fAlpha = fUpsampledColorAndWeight.www / fAccumulation;
    fHistoryColor = ffxLerp(fHistoryColor, fUpsampledColorAndWeight.xyz, fAlpha);

    fHistoryColor = YCoCgToRGB(fHistoryColor);

#if FFX_FSR2_OPTION_HDR_COLOR_INPUT
    fHistoryColor = InverseTonemap(fHistoryColor);
#endif
}

void RectifyHistory(
    const AccumulationPassCommonParams params,
    RectificationBox clippingBox,
    FFX_PARAMETER_INOUT FfxFloat32x3 fHistoryColor,
    FFX_PARAMETER_INOUT FfxFloat32x3 fAccumulation,
    FfxFloat32 fLockContributionThisFrame,
    FfxFloat32 fTemporalReactiveFactor,
    FfxFloat32 fLumaInstabilityFactor)
{
    FfxFloat32 fScaleFactorInfluence = ffxMin(20.0f, ffxPow(FfxFloat32(1.0f / length(DownscaleFactor().x * DownscaleFactor().y)), 3.0f));

    const FfxFloat32 fVecolityFactor = ffxSaturate(params.fHrVelocity / 20.0f);
    const FfxFloat32 fBoxScaleT = ffxMax(params.fDepthClipFactor, ffxMax(params.fAccumulationMask, fVecolityFactor));
    FfxFloat32 fBoxScale = ffxLerp(fScaleFactorInfluence, 1.0f, fBoxScaleT);

    FfxFloat32x3 fScaledBoxVec = clippingBox.boxVec * fBoxScale;
    FfxFloat32x3 boxMin = clippingBox.boxCenter - fScaledBoxVec;
    FfxFloat32x3 boxMax = clippingBox.boxCenter + fScaledBoxVec;
    FfxFloat32x3 boxCenter = clippingBox.boxCenter;
    FfxFloat32 boxVecSize = length(clippingBox.boxVec);

    boxMin = ffxMax(clippingBox.aabbMin, boxMin);
    boxMax = ffxMin(clippingBox.aabbMax, boxMax);

    // [FSR 4.0] Aggressive Alpha Ghosting Rejection (The Cleaner)
    // If we have a reactive surface (alpha/transparency) moving fast, tighten the box to kill trails.
    // This addresses the "Atreus hair" ghosting issue.
    if (params.fDilatedReactiveFactor > 0.1f && params.fHrVelocity > 2.0f) {
         const FfxFloat32 fGhostKiller = ffxSaturate((params.fHrVelocity - 2.0f) / 10.0f) * params.fDilatedReactiveFactor;
         // Pull the bounding box tighter towards the center (current frame)
         // [FSR Clarity] Use tunable aggression
         boxMin = ffxLerp(boxMin, boxCenter, fGhostKiller * ClarityAlphaAggression()); 
         boxMax = ffxLerp(boxMax, boxCenter, fGhostKiller * ClarityAlphaAggression()); 
    }

    if (any(FFX_GREATER_THAN(boxMin, fHistoryColor)) || any(FFX_GREATER_THAN(fHistoryColor, boxMax))) {

        const FfxFloat32x3 fClampedHistoryColor = clamp(fHistoryColor, boxMin, boxMax);

        FfxFloat32x3 fHistoryContribution = ffxMax(fLumaInstabilityFactor, fLockContributionThisFrame).xxx;
        
        const FfxFloat32 fReactiveFactor = params.fDilatedReactiveFactor;
        const FfxFloat32 fReactiveContribution = 1.0f - ffxPow(fReactiveFactor, 1.0f / 2.0f);
        fHistoryContribution *= fReactiveContribution;

        // Scale history color using rectification info, also using accumulation mask to avoid potential invalid color protection
        fHistoryColor = ffxLerp(fClampedHistoryColor, fHistoryColor, ffxSaturate(fHistoryContribution));

        // Scale accumulation using rectification info
        const FfxFloat32x3 fAccumulationMin = ffxMin(fAccumulation, FFX_BROADCAST_FLOAT32X3(0.1f));
        fAccumulation = ffxLerp(fAccumulationMin, fAccumulation, ffxSaturate(fHistoryContribution));
    }
    
    // [FSR Clarity] Disocclusion Management (Ghosting)
    // If depth clip factor indicates significant disocclusion, we shouldn't trust history at all.
    // The original FSR might still blend a bit. We want to force it to use the current frame.
    // [FSR Clarity] Use tunable threshold
    if (params.fDepthClipFactor > ClarityDepthThreshold()) {
        // High depth clip factor means the reprojected depth is very different from current depth.
        // This is a disocclusion event.
        // Force history to match current frame (boxCenter is essentially the current frame color)
        fHistoryColor = clippingBox.boxCenter;
        // Reset accumulation to start fresh
        // [FSR Clarity] Smart Fade: Smooth transition instead of hard reset
        fAccumulation = ffxLerp(fAccumulation, FFX_BROADCAST_FLOAT32X3(0.0f), 0.5f); 
    }
}

void WriteUpscaledOutput(FfxInt32x2 iPxHrPos, FfxFloat32x3 fUpscaledColor)
{
    StoreUpscaledOutput(iPxHrPos, fUpscaledColor);
}

void FinalizeLockStatus(const AccumulationPassCommonParams params, FfxFloat32x2 fLockStatus, FfxFloat32 fUpsampledWeight)
{
    // we expect similar motion for next frame
    // kill lock if that location is outside screen, avoid locks to be clamped to screen borders
    FfxFloat32x2 fEstimatedUvNextFrame = params.fHrUv - params.fMotionVector;
    if (IsUvInside(fEstimatedUvNextFrame) == false) {
        KillLock(fLockStatus);
    }
    else {
        // Decrease lock lifetime
        const FfxFloat32 fLifetimeDecreaseLanczosMax = FfxFloat32(JitterSequenceLength()) * FfxFloat32(fAverageLanczosWeightPerFrame);
        const FfxFloat32 fLifetimeDecrease = FfxFloat32(fUpsampledWeight / fLifetimeDecreaseLanczosMax);
        fLockStatus[LOCK_LIFETIME_REMAINING] = ffxMax(FfxFloat32(0), fLockStatus[LOCK_LIFETIME_REMAINING] - fLifetimeDecrease);
    }

    StoreLockStatus(params.iPxHrPos, fLockStatus);
}


FfxFloat32x3 ComputeBaseAccumulationWeight(const AccumulationPassCommonParams params, FfxFloat32 fThisFrameReactiveFactor, FfxBoolean bInMotionLastFrame, FfxFloat32 fUpsampledWeight, LockState lockState, FfxFloat32 fLumaInstabilityFactor)
{
    // Always assume max accumulation was reached
    FfxFloat32 fBaseAccumulation = fMaxAccumulationLanczosWeight * FfxFloat32(params.bIsExistingSample) * (1.0f - fThisFrameReactiveFactor) * (1.0f - params.fDepthClipFactor);

    fBaseAccumulation = ffxMin(fBaseAccumulation, ffxLerp(fBaseAccumulation, fUpsampledWeight * 10.0f, ffxMax(FfxFloat32(bInMotionLastFrame), ffxSaturate(params.fHrVelocity * FfxFloat32(10)))));

    fBaseAccumulation = ffxMin(fBaseAccumulation, ffxLerp(fBaseAccumulation, fUpsampledWeight, ffxSaturate(params.fHrVelocity / FfxFloat32(20))));

    // [FSR Clarity] Alpha Stability (Hair/Foliage)
    // If this is a reactive pixel (hair/alpha) AND we detect instability (luma variance),
    // we should NOT accumulate. Trusting history here causes smearing/ghosting.
    // We use a heuristic: Reactive Factor > Threshold AND Luma Instability.
    if (fThisFrameReactiveFactor > 0.5f) {
         // [FSR Clarity] Adaptive Minimum: Don't force 0.0f, use adaptive minimum based on luma instability
         // This prevents dither/noise.
         fBaseAccumulation = ffxMin(fBaseAccumulation, ffxLerp(0.01f, 0.1f, ffxSaturate(fLumaInstabilityFactor)));
    }

    // [FSR 4.0] Temporal Stability for Thin Features (The Stabilizer)
    // If this pixel was locked (thin feature), trust history more to prevent flickering.
    // This addresses the "Atreus hair" flickering issue.
    if (lockState.WasLockedPrevFrame) {
        // Only stabilize if motion is relatively low to avoid smearing. 
        // We fade out the stability boost as velocity increases.
        FfxFloat32 fStabilityFactor = 1.0f - ffxSaturate(params.fHrVelocity / 4.0f); 
        // Boost accumulation factor to 0.90 (very high history trust) when stable
        fBaseAccumulation = ffxMax(fBaseAccumulation, 0.90f * fStabilityFactor);
    }

    return fBaseAccumulation.xxx;
}

FfxFloat32 ComputeLumaInstabilityFactor(const AccumulationPassCommonParams params, RectificationBox clippingBox, FfxFloat32 fThisFrameReactiveFactor, FfxFloat32 fLuminanceDiff)
{
    const FfxFloat32 fUnormThreshold = 1.0f / 255.0f;
    const FfxInt32 N_MINUS_1 = 0;
    const FfxInt32 N_MINUS_2 = 1;
    const FfxInt32 N_MINUS_3 = 2;
    const FfxInt32 N_MINUS_4 = 3;

    FfxFloat32 fCurrentFrameLuma = clippingBox.boxCenter.x;

#if FFX_FSR2_OPTION_HDR_COLOR_INPUT
    fCurrentFrameLuma = fCurrentFrameLuma / (1.0f + ffxMax(0.0f, fCurrentFrameLuma));
#endif

    fCurrentFrameLuma = round(fCurrentFrameLuma * 255.0f) / 255.0f;

    const FfxBoolean bSampleLumaHistory = (ffxMax(ffxMax(params.fDepthClipFactor, params.fAccumulationMask), fLuminanceDiff) < 0.1f) && (params.bIsNewSample == false);
    FfxFloat32x4 fCurrentFrameLumaHistory = bSampleLumaHistory ? SampleLumaHistory(params.fReprojectedHrUv) : FFX_BROADCAST_FLOAT32X4(0.0f);

    FfxFloat32 fLumaInstability = 0.0f;
    FfxFloat32 fDiffs0 = (fCurrentFrameLuma - fCurrentFrameLumaHistory[N_MINUS_1]);

    FfxFloat32 fMin = abs(fDiffs0);

    if (fMin >= fUnormThreshold)
    {
        for (int i = N_MINUS_2; i <= N_MINUS_4; i++) {
            FfxFloat32 fDiffs1 = (fCurrentFrameLuma - fCurrentFrameLumaHistory[i]);

            if (sign(fDiffs0) == sign(fDiffs1)) {
                
                // Scale difference to protect historically similar values
                const FfxFloat32 fMinBias = 1.0f;
                fMin = ffxMin(fMin, abs(fDiffs1) * fMinBias);
            }
        }

        const FfxFloat32 fBoxSize = clippingBox.boxVec.x;
        const FfxFloat32 fBoxSizeFactor = ffxPow(ffxSaturate(fBoxSize / 0.1f), 6.0f);

        fLumaInstability = FfxFloat32(fMin != abs(fDiffs0)) * fBoxSizeFactor;
        fLumaInstability = FfxFloat32(fLumaInstability > fUnormThreshold);

        fLumaInstability *= 1.0f - ffxMax(params.fAccumulationMask, ffxPow(fThisFrameReactiveFactor, 1.0f / 6.0f));
    }

    //shift history
    fCurrentFrameLumaHistory[N_MINUS_4] = fCurrentFrameLumaHistory[N_MINUS_3];
    fCurrentFrameLumaHistory[N_MINUS_3] = fCurrentFrameLumaHistory[N_MINUS_2];
    fCurrentFrameLumaHistory[N_MINUS_2] = fCurrentFrameLumaHistory[N_MINUS_1];
    fCurrentFrameLumaHistory[N_MINUS_1] = fCurrentFrameLuma;

    StoreLumaHistory(params.iPxHrPos, fCurrentFrameLumaHistory);

    return fLumaInstability * FfxFloat32(fCurrentFrameLumaHistory[N_MINUS_4] != 0);
}

FfxFloat32 ComputeTemporalReactiveFactor(const AccumulationPassCommonParams params, FfxFloat32 fTemporalReactiveFactor)
{
    FfxFloat32 fNewFactor = ffxMin(0.99f, fTemporalReactiveFactor);

    fNewFactor = ffxMax(fNewFactor, ffxLerp(fNewFactor, 0.4f, ffxSaturate(params.fHrVelocity)));

    fNewFactor = ffxMax(fNewFactor * fNewFactor, ffxMax(params.fDepthClipFactor * 0.1f, params.fDilatedReactiveFactor));

    // Force reactive factor for new samples
    fNewFactor = params.bIsNewSample ? 1.0f : fNewFactor;

    if (ffxSaturate(params.fHrVelocity * 10.0f) >= 1.0f) {
        fNewFactor = ffxMax(FSR2_EPSILON, fNewFactor) * -1.0f;
    }
    
    return fNewFactor;
}

AccumulationPassCommonParams InitParams(FfxInt32x2 iPxHrPos)
{
    AccumulationPassCommonParams params;

    params.iPxHrPos = iPxHrPos;
    const FfxFloat32x2 fHrUv = (iPxHrPos + 0.5f) / DisplaySize();
    params.fHrUv = fHrUv;
    
    const FfxFloat32x2 fLrUvJittered = fHrUv + Jitter() / RenderSize();
    params.fLrUv_HwSampler = ClampUv(fLrUvJittered, RenderSize(), MaxRenderSize());

    params.fLrUv_HwSampler = ClampUv(fLrUvJittered, RenderSize(), MaxRenderSize());

    // [FSR Clarity] Use Filtered Motion Vector
    params.fMotionVector = GetFilteredMotionVector(iPxHrPos, fHrUv);
    params.fHrVelocity = GetPxHrVelocity(params.fMotionVector);

    ComputeReprojectedUVs(params, params.fReprojectedHrUv, params.bIsExistingSample);

    params.fDepthClipFactor = ffxSaturate(SampleDepthClip(params.fLrUv_HwSampler));
    
    const FfxFloat32x2 fDilatedReactiveMasks = SampleDilatedReactiveMasks(params.fLrUv_HwSampler);
    params.fDilatedReactiveFactor = fDilatedReactiveMasks.x;
    params.fAccumulationMask = fDilatedReactiveMasks.y;
    params.bIsResetFrame = (0 == FrameIndex());

    params.bIsNewSample = (params.bIsExistingSample == false || params.bIsResetFrame);

    return params;
}

void Accumulate(FfxInt32x2 iPxHrPos)
{
    const AccumulationPassCommonParams params = InitParams(iPxHrPos);

    FfxFloat32x3 fHistoryColor = FfxFloat32x3(0, 0, 0);
    FfxFloat32x2 fLockStatus;
    InitializeNewLockSample(fLockStatus);

    FfxFloat32 fTemporalReactiveFactor = 0.0f;
    FfxBoolean bInMotionLastFrame = FFX_FALSE;
    LockState lockState = { FFX_FALSE , FFX_FALSE };
    if (params.bIsExistingSample && !params.bIsResetFrame) {
        ReprojectHistoryColor(params, fHistoryColor, fTemporalReactiveFactor, bInMotionLastFrame);
        lockState = ReprojectHistoryLockStatus(params, fLockStatus);
    }

    FfxFloat32 fThisFrameReactiveFactor = ffxMax(params.fDilatedReactiveFactor, fTemporalReactiveFactor);

    FfxFloat32 fLuminanceDiff = 0.0f;
    FfxFloat32 fLockContributionThisFrame = 0.0f;
    UpdateLockStatus(params, fThisFrameReactiveFactor, lockState, fLockStatus, fLockContributionThisFrame, fLuminanceDiff);

    // Load upsampled input color
    RectificationBox clippingBox;
    FfxFloat32x4 fUpsampledColorAndWeight = ComputeUpsampledColorAndWeight(params, clippingBox, fThisFrameReactiveFactor);
    
    const FfxFloat32 fLumaInstabilityFactor = ComputeLumaInstabilityFactor(params, clippingBox, fThisFrameReactiveFactor, fLuminanceDiff);


    FfxFloat32x3 fAccumulation = ComputeBaseAccumulationWeight(params, fThisFrameReactiveFactor, bInMotionLastFrame, fUpsampledColorAndWeight.w, lockState, fLumaInstabilityFactor);

    if (params.bIsNewSample) {
        fHistoryColor = YCoCgToRGB(fUpsampledColorAndWeight.xyz);
    }
    else {
        RectifyHistory(params, clippingBox, fHistoryColor, fAccumulation, fLockContributionThisFrame, fThisFrameReactiveFactor, fLumaInstabilityFactor);

        Accumulate(params, fHistoryColor, fAccumulation, fUpsampledColorAndWeight);
    }

    fHistoryColor = UnprepareRgb(fHistoryColor, Exposure());

    FinalizeLockStatus(params, fLockStatus, fUpsampledColorAndWeight.w);

    // Get new temporal reactive factor
    fTemporalReactiveFactor = ComputeTemporalReactiveFactor(params, fThisFrameReactiveFactor);

    StoreInternalColorAndWeight(iPxHrPos, FfxFloat32x4(fHistoryColor, fTemporalReactiveFactor));

    // Output final color when RCAS is disabled
#if FFX_FSR2_OPTION_APPLY_SHARPENING == 0
    WriteUpscaledOutput(iPxHrPos, fHistoryColor);
#endif
    StoreNewLocks(iPxHrPos, 0);
}

#endif // FFX_FSR2_ACCUMULATE_H
