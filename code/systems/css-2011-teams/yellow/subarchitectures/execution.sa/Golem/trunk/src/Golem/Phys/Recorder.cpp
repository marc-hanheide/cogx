/** @file Recorder.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/Stream.h>
#include <Golem/Phys/Recorder.h>
#include <Golem/Phys/Msg.h>

//------------------------------------------------------------------------------

#ifdef WIN32
using namespace std;

#include <unknwn.h>
#include <gdiplus.h>
#include <sstream>
#include <iomanip>

using namespace Gdiplus;
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Recorder::Recorder(golem::Context& context) : context(context) {
}

Recorder::~Recorder() {
	bTerminate = true;
	ev.set(true);
	if (!thread.join(threadTimeOut))
		context.getMessageStream()->write(Message::LEVEL_ERROR, "~Recorder(): Cannot stop recorder thread"	);
}

bool Recorder::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgRecorderInvalidDesc(Message::LEVEL_CRIT, "Recorder::create(): Invalid description");
	
	fileName = desc.fileName;
	fileExt = desc.fileExt;
	leadingZeros = desc.leadingZeros;
	encoderString = desc.encoderString;
	fileCount = 1;
	
	bufferLen = desc.bufferLen;
	frameSeq.clear();

	threadTimeOut = desc.threadTimeOut;
	bTerminate = false;
	if (!thread.start(this))
		throw MsgRecorderThreadLaunch(Message::LEVEL_CRIT, "Recorder::create(): Unable to launch Recorder thread");

	return true;
}

#ifdef WIN32
void Recorder::save(const Frame& frame) {
	// Initialize GDI+
	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
	
	{
		Bitmap bitmap(frame.width, frame.height, PixelFormat32bppARGB);
		Rect rect1(0, 0, frame.width, frame.height);

		BitmapData bitmapData;
		memset(&bitmapData, 0, sizeof(bitmapData));
		bitmap.LockBits(&rect1, ImageLockModeRead, PixelFormat32bppARGB, &bitmapData);

		UINT* pixels = (UINT*)bitmapData.Scan0;
		if (pixels == NULL) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Recorder::save(): Unable to allocate bitmap");
			return;
		}

		INT stride1 = bitmapData.Stride;
		if (stride1 < 0)
			stride1 = -stride1;

		for(UINT i = 0; i < bitmapData.Height; i++)
			for(UINT j = 0; j < bitmapData.Width; j++)
				pixels[i*stride1/4 + j] = frame.data[i*frame.width + j];

		bitmap.UnlockBits(&bitmapData);
		bitmap.RotateFlip(RotateNoneFlipY);
		
		UINT num = 0; // number of image encoders
		UINT size = 0; // size of the image encoder array in bytes
		GetImageEncodersSize(&num, &size);
		if (size <= 0) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Recorder::save(): Unable to find any image encoders");
			return;
		}

		golem::shared_ptr<U8, arr_cnt<U8> > pImageCodecInfoBuff(new U8 [size]);
		if (pImageCodecInfoBuff == NULL) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Recorder::save(): Unable to allocate encoder info");
			return;
		}

		ImageCodecInfo* pImageCodecInfo = (ImageCodecInfo*)pImageCodecInfoBuff.get();
		std::wstring wEncoderString(encoderString.begin(), encoderString.end());
		CLSID Clsid;
		bool bRes = false;
		GetImageEncoders(num, size, pImageCodecInfo);
		for(UINT j = 0; j < num; j++)
			if (wcscmp(pImageCodecInfo[j].MimeType, wEncoderString.c_str()) == 0) {
				Clsid = pImageCodecInfo[j].Clsid;
				bRes = true;
				break;
			}

		if (!bRes) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Recorder::save(): Unable to find image encoder");
			return;
		}

		std::ostringstream ostr(std::ostringstream::out);
		if (leadingZeros)
			ostr << fileName.c_str() << std::setw(6) << std::setfill('0') << fileCount << fileExt.c_str();
		else
			ostr << fileName.c_str() << fileCount << fileExt.c_str();
		

		std::string path(ostr.str());
		std::wstring wPath(path.begin(), path.end());
		golem::mkdir(path.c_str()); // make sure that the directory exists
		Status status = bitmap.Save(wPath.c_str(), &Clsid);
		if (status != Gdiplus::Ok) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Recorder::save(): Unable to save image");
			return;
		}

		fileCount++;
	}

	// Shutdown GDI+
	GdiplusShutdown(gdiplusToken);
}
#else
void Recorder::save(const Frame& frame) {
	// TODO
}
#endif

void Recorder::run() {
	for (;;) {
		bool bRet = ev.wait();
		if (bTerminate)
			break;
		if (!bRet)
			continue;

		Frame frame;
		{
			CriticalSectionWrapper csw(cs);
			if (frameSeq.size() <= 0) {
				ev.set(false);
				continue;
			}
			frame = frameSeq.front();
			frameSeq.pop_front();
		}
		save(frame);
	}
}

void Recorder::capture(int x, int y, int width, int height) {
	{
		CriticalSectionWrapper csw(cs);
		if (frameSeq.size() >= bufferLen)
			return;
	}

	Frame frame(x, y, width, height);
	if (frame.data == NULL) {
		context.getMessageStream()->write(Message::LEVEL_ERROR,"Recorder::capture(): Unable to allocate new screen frame");
		return;
	}

	::glFlush();
	::glFinish();
	::glReadPixels(x, y, width, height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, frame.data.get());

	{
		CriticalSectionWrapper csw(cs);
		frameSeq.push_back(frame);
		ev.set(true);
	}
}

//------------------------------------------------------------------------------
