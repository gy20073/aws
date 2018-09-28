#include "ScreenCapturer.h"
#include "lib/natives.h"
#include <stdlib.h>
#include <ctime>

#include "lib/script.h"
#include "Server.h"


ScreenCapturer::ScreenCapturer(int r_w, int r_h){
	// Hard encoded system framesize
	imageWidth = 1920;
	imageHeight = 1080;

	resize_w = r_w;
	resize_h = r_h;

	count = 0;

	// Round up the scan line size to a multiple of 4
	length = ((imageWidth * 3 + 3) / 4 * 4) * imageHeight;
	length_resized = ((resize_w * 3 + 3) / 4 * 4) * resize_h;

	//Screen capture buffer
	GRAPHICS::_GET_SCREEN_ACTIVE_RESOLUTION(&windowWidth, &windowHeight);
	//hWnd = ::FindWindow(NULL, "Grand Theft Auto V");
	hWindowDC = GetDC(NULL);
	hCaptureDC = CreateCompatibleDC(hWindowDC);
	hCaptureBitmap = CreateCompatibleBitmap(hWindowDC, imageWidth, imageHeight);
	SelectObject(hCaptureDC, hCaptureBitmap);
	SetStretchBltMode(hCaptureDC, COLORONCOLOR);

	pixels = (UINT8*)malloc(length);
	pixels_resized = (UINT8*)malloc(length_resized);
	info.biSize = sizeof(BITMAPINFOHEADER);
	info.biPlanes = 1;
	info.biBitCount = 24;
	info.biWidth = resize_w;
	info.biHeight = -resize_h;
	info.biCompression = BI_RGB;
	info.biSizeImage = 0;

	// D3D method
	/*
	ZeroMemory(&d3dpp, sizeof(D3DPRESENT_PARAMETERS));
	ZeroMemory(&d3dpp, sizeof(d3dpp));

	g_pD3D = Direct3DCreate9(D3D_SDK_VERSION);
	g_pD3D->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &ddm);
	d3dpp.Windowed = TRUE;
	d3dpp.Flags = D3DPRESENTFLAG_LOCKABLE_BACKBUFFER;
	d3dpp.BackBufferFormat = ddm.Format;
	d3dpp.BackBufferHeight = ddm.Height;
	d3dpp.BackBufferWidth = ddm.Width;
	d3dpp.MultiSampleType = D3DMULTISAMPLE_NONE;
	d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
	d3dpp.hDeviceWindow = GetDesktopWindow();
	d3dpp.PresentationInterval = D3DPRESENT_INTERVAL_DEFAULT;
	d3dpp.FullScreen_RefreshRateInHz = D3DPRESENT_RATE_DEFAULT;

	hr = g_pD3D->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, GetDesktopWindow(), D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, &g_pd3dDevice);
	hr = g_pd3dDevice->GetRenderTarget(0, &pRenderTarget);
	hr = g_pd3dDevice->CreateOffscreenPlainSurface(ddm.Width, ddm.Height, D3DFMT_A8R8G8B8, D3DPOOL_SCRATCH, &pSurface, NULL);	
	*/




	// DESKTOP DUPLICATE
	gDriverTypes = D3D_DRIVER_TYPE_HARDWARE;

	gNumDriverTypes = 1;// ARRAYSIZE(gDriverTypes);
		D3D_FEATURE_LEVEL gFeatureLevels[] =
		{
			D3D_FEATURE_LEVEL_11_0,
			D3D_FEATURE_LEVEL_10_1,
			D3D_FEATURE_LEVEL_10_0,
			D3D_FEATURE_LEVEL_9_1
		};
		gNumFeatureLevels = ARRAYSIZE(gFeatureLevels);
		hr2 = E_FAIL;
		lresult = -1;

		// Create device
		for (UINT DriverTypeIndex = 0; DriverTypeIndex < gNumDriverTypes; ++DriverTypeIndex)
		{
			hr2 = D3D11CreateDevice(
				nullptr,
				gDriverTypes,
				nullptr,
				0,
				gFeatureLevels,
				gNumFeatureLevels,
				D3D11_SDK_VERSION,
				&lDevice,
				&lFeatureLevel,
				&lImmediateContext);

			if (SUCCEEDED(hr2))
			{
				// Device creation success, no need to loop anymore
				printf("\n Device creation success for Dup API");
				break;
			}

			lDevice.Release();

			lImmediateContext.Release();
		}

		// Get DXGI device
		hr2 = lDevice->QueryInterface(IID_PPV_ARGS(&lDxgiDevice));

		if (FAILED(hr2)){
			printf("\n Get DXGI device failed");
		}
		// Get DXGI adapter
		hr2 = lDxgiDevice->GetParent(
			__uuidof(IDXGIAdapter),
			reinterpret_cast<void**>(&lDxgiAdapter));
		if (FAILED(hr2)){
			printf("\n Get DXGI adapter failed");
		}
		lDxgiDevice.Release();

		// Get output
		hr2 = lDxgiAdapter->EnumOutputs(
			Output,
			&lDxgiOutput);
		if (FAILED(hr2)){
			printf("\n Get output1 failed");
		}
		lDxgiAdapter.Release();

		hr2 = lDxgiOutput->GetDesc(
			&lOutputDesc);
		if (FAILED(hr2)){
			printf("\n Get output2 failed");
		}
		// QI for Output 1


		hr2 = lDxgiOutput->QueryInterface(IID_PPV_ARGS(&lDxgiOutput1));
		if (FAILED(hr2)){
			printf("\n QI for Output 1 failed");
		}
		lDxgiOutput.Release();

		// Create desktop duplication
		hr2 = lDxgiOutput1->DuplicateOutput(
			lDevice,
			&lDeskDupl);
		if (FAILED(hr2)){
			printf("\n Create desktop duplication failed");
		}
		lDxgiOutput1.Release();

		// Create GUI drawing texture
		lDeskDupl->GetDesc(&lOutputDuplDesc);
		desc.Width = lOutputDuplDesc.ModeDesc.Width;
		desc.Height = lOutputDuplDesc.ModeDesc.Height;
		desc.Format = lOutputDuplDesc.ModeDesc.Format;
		desc.ArraySize = 1;
		desc.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_RENDER_TARGET;
		desc.MiscFlags = D3D11_RESOURCE_MISC_GDI_COMPATIBLE;
		desc.SampleDesc.Count = 1;
		desc.SampleDesc.Quality = 0;
		desc.MipLevels = 1;
		desc.CPUAccessFlags = 0;
		desc.Usage = D3D11_USAGE_DEFAULT;
		hr2 = lDevice->CreateTexture2D(&desc, NULL, &lGDIImage);

		// Create CPU access texture
		desc.Width = lOutputDuplDesc.ModeDesc.Width;
		desc.Height = lOutputDuplDesc.ModeDesc.Height;
		desc.Format = lOutputDuplDesc.ModeDesc.Format;
		desc.ArraySize = 1;
		desc.BindFlags = 0;
		desc.MiscFlags = 0;
		desc.SampleDesc.Count = 1;
		desc.SampleDesc.Quality = 0;
		desc.MipLevels = 1;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ | D3D11_CPU_ACCESS_WRITE;
		desc.Usage = D3D11_USAGE_STAGING;
		hr2 = lDevice->CreateTexture2D(&desc, NULL, &lDestImage);

		if (FAILED(hr2)){
			printf("\n Create CPU access texture failed");
		}


		
		subresource = D3D11CalcSubresource(0, 0, 0);
		lImmediateContext->Map(lDestImage, subresource, D3D11_MAP_READ_WRITE, 0, &resource);

		// BMP 32 bpp
		ZeroMemory(&lBmpInfo, sizeof(BITMAPINFO));
		lBmpInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
		lBmpInfo.bmiHeader.biBitCount = 32;
		lBmpInfo.bmiHeader.biCompression = BI_RGB;
		lBmpInfo.bmiHeader.biWidth = lOutputDuplDesc.ModeDesc.Width;
		lBmpInfo.bmiHeader.biHeight = lOutputDuplDesc.ModeDesc.Height;
		lBmpInfo.bmiHeader.biPlanes = 1;
		lBmpInfo.bmiHeader.biSizeImage = lOutputDuplDesc.ModeDesc.Width
			* lOutputDuplDesc.ModeDesc.Height * 4;


		bmpFileHeader.bfReserved1 = 0;
		bmpFileHeader.bfReserved2 = 0;
		bmpFileHeader.bfSize = sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER)+lBmpInfo.bmiHeader.biSizeImage;
		bmpFileHeader.bfType = 'MB';
		bmpFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER);

		
		freopen("outer_script.log", "w", stdout);
		
		hr2 = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);

		g_MFEncoder = new MF_H264_Encoder(lDevice, lImmediateContext);
		if (!g_MFEncoder->Init()) { printf("Failed to init Media Foundation H.264 Encoder!\n"); }

		hWindowDC2 = GetDC(NULL);
		hCaptureDC2 = CreateCompatibleDC(hWindowDC2);

		pBuf = (std::unique_ptr<BYTE>)new BYTE[lBmpInfo.bmiHeader.biSizeImage];
		hBitmapTexture = CreateCompatibleBitmap(hWindowDC, imageWidth, imageHeight);
		resized_bitmap = CreateCompatibleBitmap(hWindowDC2, resize_w, resize_h);
		// Select bitmaps into devices
		SelectObject(hCaptureDC, hBitmapTexture);
		SelectObject(hCaptureDC2, resized_bitmap);

		lBmpRowPitch = lOutputDuplDesc.ModeDesc.Width * 4;
		lRowPitch = std::min<UINT>(lBmpRowPitch, resource.RowPitch);

		// No color distortion
		SetStretchBltMode(hCaptureDC2, HALFTONE);
		SetStretchBltMode(hCaptureDC, HALFTONE);

}

ScreenCapturer::~ScreenCapturer(){
	free(pixels);
	free(pixels_resized);
	ReleaseDC(hWnd, hWindowDC);
	DeleteDC(hCaptureDC);
	DeleteObject(hCaptureBitmap);

	
}

void ScreenCapturer::finishVid() {
	g_MFEncoder->Shutdown();
	delete g_MFEncoder;
	g_MFEncoder = nullptr;
	CoUninitialize;

}


void ScreenCapturer::capture() {
		// Get new frame
		hr2 = lDeskDupl->AcquireNextFrame(500, &lFrameInfo, &lDesktopResource);
		if (FAILED(hr2)){
			printf("\n---->Dup Fail");
			if ((hr2 != DXGI_ERROR_ACCESS_LOST) && (hr2 != DXGI_ERROR_WAIT_TIMEOUT)) {
				printf("Failed to acquire next frame in DUPLICATIONMANAGER Error %X", hr2);
			}
		}
		
		hr2 = lDesktopResource->QueryInterface(IID_PPV_ARGS(&lAcquiredDesktopImage));
		if (FAILED(hr2))
			printf("\n---->QueryInterface Fail");
		if (lAcquiredDesktopImage == nullptr)
			printf("\n---->lAcquiredDesktopImage Fail");

		// write video
		if (saveVid)
			g_MFEncoder->WriteFrame(lAcquiredDesktopImage);

		// Copy image into GDI drawing texture
		lImmediateContext->CopyResource(lGDIImage, lAcquiredDesktopImage);
		lImmediateContext->CopyResource(lDestImage, lGDIImage);

		sptr = reinterpret_cast<BYTE*>(resource.pData);
		dptr = pBuf.get() + lBmpInfo.bmiHeader.biSizeImage - lBmpRowPitch;
		for (size_t h = 0; h < lOutputDuplDesc.ModeDesc.Height; ++h) {
			memcpy_s(dptr, lBmpRowPitch, sptr, lRowPitch);
			sptr += resource.RowPitch;
			dptr -= lBmpRowPitch;
		}
		SetBitmapBits(hBitmapTexture, desc.Width*desc.Height * 4, pBuf.get());

		// Resize bitmap
		StretchBlt(hCaptureDC2, 0, 0, resize_w, resize_h, hCaptureDC, 0, 0, imageWidth, imageHeight, SRCCOPY);
		// Get bitmap data
		GetDIBits(hCaptureDC2, resized_bitmap, 0, resize_h, pixels_resized, (BITMAPINFO*)&info, DIB_RGB_COLORS);
		
		// clean up
		hr2 = lDeskDupl->ReleaseFrame();	
		// lGDIImage->Release(); lDestImage->Release();
		lAcquiredDesktopImage->Release(); lDesktopResource->Release();
		// free(dptr); free(sptr); //x

}


void ScreenCapturer::capture_part1() {
	// Get new frame
	hr2 = lDeskDupl->AcquireNextFrame(500, &lFrameInfo, &lDesktopResource);
	if (FAILED(hr2)) {
		printf("\n---->Dup Fail");
		if ((hr2 != DXGI_ERROR_ACCESS_LOST) && (hr2 != DXGI_ERROR_WAIT_TIMEOUT)) {
			printf("Failed to acquire next frame in DUPLICATIONMANAGER Error %X", hr2);
		}
	}
}

void ScreenCapturer::capture_part2() {
	hr2 = lDesktopResource->QueryInterface(IID_PPV_ARGS(&lAcquiredDesktopImage));
	if (FAILED(hr2))
		printf("\n---->QueryInterface Fail");
	if (lAcquiredDesktopImage == nullptr)
		printf("\n---->lAcquiredDesktopImage Fail");

	// write video
	if (saveVid)
		g_MFEncoder->WriteFrame(lAcquiredDesktopImage);

	// Copy image into GDI drawing texture
	lImmediateContext->CopyResource(lGDIImage, lAcquiredDesktopImage);
	lImmediateContext->CopyResource(lDestImage, lGDIImage);

	sptr = reinterpret_cast<BYTE*>(resource.pData);
	dptr = pBuf.get() + lBmpInfo.bmiHeader.biSizeImage - lBmpRowPitch;
	for (size_t h = 0; h < lOutputDuplDesc.ModeDesc.Height; ++h) {
		memcpy_s(dptr, lBmpRowPitch, sptr, lRowPitch);
		sptr += resource.RowPitch;
		dptr -= lBmpRowPitch;
	}
	SetBitmapBits(hBitmapTexture, desc.Width*desc.Height * 4, pBuf.get());

	// Resize bitmap
	StretchBlt(hCaptureDC2, 0, 0, resize_w, resize_h, hCaptureDC, 0, 0, imageWidth, imageHeight, SRCCOPY);
	// Get bitmap data
	GetDIBits(hCaptureDC2, resized_bitmap, 0, resize_h, pixels_resized, (BITMAPINFO*)&info, DIB_RGB_COLORS);

	// clean up
	hr2 = lDeskDupl->ReleaseFrame();
	// lGDIImage->Release(); lDestImage->Release();
	lAcquiredDesktopImage->Release(); lDesktopResource->Release();
	// free(dptr); free(sptr); //x

}

