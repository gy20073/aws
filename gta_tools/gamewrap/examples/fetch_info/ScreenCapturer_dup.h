#pragma once

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
//#include <d3d9.h>
//#include <d3dx9.h>

#include "CComPtrCustom.h"

#include "mf_h264_encoder.h"

#define WIN32_LEAN_AND_MEAN    

#include <windows.h>
#include <shlobj.h>
#include <shellapi.h>
#include <dxgi1_2.h>
#include <d3d11.h>
#include <memory>
#include <algorithm>
#include <string>

//#pragma comment(lib, "d3d9.lib")
//#pragma comment(lib, "d3dx9.lib")
#pragma comment(lib, "D3D11.lib")
#pragma comment(lib, "D3DCompiler.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")




class ScreenCapturer {
private:
	bool saveVid = true;

	int windowWidth;
	int windowHeight;
	int resize_w;
	int resize_h;

	int count;
	HWND hWnd;
	HDC hWindowDC;
	HDC hCaptureDC;
	HBITMAP hCaptureBitmap;
	HBITMAP tran_bitmap;
	HBITMAP resized_bitmap;
	BITMAPINFOHEADER info;
	
	// Swith between different methods
	// bool useD3D = false;
	// bool useGDI = false;
	// bool useDeskDup = true; // DEFAULT

	/*
	LPDIRECT3D9 g_pD3D = NULL;
	D3DDISPLAYMODE ddm;
	D3DPRESENT_PARAMETERS d3dpp;
	
	IDirect3DSurface9 * pSurface;
	HRESULT hr;
	IDirect3DSurface9* pRenderTarget = NULL;
	IDirect3DDevice9 * g_pd3dDevice;
	*/

public:
	int imageWidth;
	int imageHeight;
	int length;
	int length_resized;

	UINT8* pixels;

	UINT8* pixels_resized;


	ScreenCapturer(int frameWidth, int frameHeight);
	~ScreenCapturer();
	void capture();
	void capture_part1();
	void capture_part2();
	void finishVid();
	HBITMAP ScaleBitmapInt(HBITMAP hBmp,
		WORD wNewWidth,
		WORD wNewHeight);




	// Driver types supported
	D3D_DRIVER_TYPE gDriverTypes;
	UINT gNumDriverTypes;

	// Feature levels supported
	D3D_FEATURE_LEVEL *gFeatureLevels;

	UINT gNumFeatureLevels;

	// Desktop Duplication API
	CComPtrCustom<ID3D11Device> lDevice;
	CComPtrCustom<ID3D11DeviceContext> lImmediateContext;
	CComPtrCustom<IDXGIOutputDuplication> lDeskDupl;
	CComPtrCustom<ID3D11Texture2D> lAcquiredDesktopImage;
	CComPtrCustom<ID3D11Texture2D> lGDIImage;
	CComPtrCustom<ID3D11Texture2D> lDestImage;
	DXGI_OUTPUT_DESC lOutputDesc;
	DXGI_OUTDUPL_DESC lOutputDuplDesc;

	D3D11_TEXTURE2D_DESC desc;
	CComPtrCustom<IDXGIOutput> lDxgiOutput;
	CComPtrCustom<IDXGIAdapter> lDxgiAdapter;
	CComPtrCustom<IDXGIDevice> lDxgiDevice;
	UINT Output = 0;

	int lresult;



	D3D_FEATURE_LEVEL lFeatureLevel;

	HRESULT hr2;

	CComPtrCustom<IDXGIOutput1> lDxgiOutput1;
	CComPtrCustom<IDXGIResource> lDesktopResource;
	DXGI_OUTDUPL_FRAME_INFO lFrameInfo;



	// BMP
	D3D11_MAPPED_SUBRESOURCE resource;
	UINT subresource;
	BITMAPINFO	lBmpInfo;

	BITMAPFILEHEADER	bmpFileHeader;

	// UINT8* toSentPtr;
	// BYTE* fileBuffer;
	// BYTE* fileBufferImg;

	// int totalFileSize;

	MF_H264_Encoder *g_MFEncoder;


	HBITMAP	hBitmapTexture = NULL;

	BYTE* sptr; BYTE* dptr;

	HDC hWindowDC2;
	HDC hCaptureDC2;

	std::unique_ptr<BYTE> pBuf;

	UINT lBmpRowPitch;
	UINT lRowPitch;
}; 





