// Copyright (C) 2023-2024 Adrian Wöltche
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see https://www.gnu.org/licenses/.

#include <format>

#include "io/memory_mapped/utils.hpp"

#if defined(MM2_WINDOWS)
#include <windows.h>
#include <fileapi.h>
#include <memoryapi.h>
#include <winioctl.h>
#elif defined(MM2_LINUX)
#include <sys/sysinfo.h>
#include <sys/mman.h>
#endif

namespace map_matching_2::io::memory_mapped {

    [[nodiscard]] std::size_t get_total_memory() {
        std::size_t memory;
#if defined(MM2_WINDOWS)
        MEMORYSTATUSEX statex;
        statex.dwLength = sizeof(statex);
        GlobalMemoryStatusEx(&statex);
        memory = statex.ullTotalPhys;
#elif defined(MM2_LINUX)
        struct sysinfo mem_info{};
        sysinfo(&mem_info);
        memory = mem_info.totalram;
        memory *= mem_info.mem_unit;
#else
#error "Can not detect total memory"
#endif
        return memory;
    }

#if defined(MM2_WINDOWS)

    SPARSE_STATUS make_sparse_file(const std::string &file) {
        DWORD fileAttributes = GetFileAttributes(file.c_str());

        if (fileAttributes == FILE_ATTRIBUTE_SPARSE_FILE) {
            return SPARSE_STATUS::IS_SPARSE_FILE;
        }

        // open file: https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilea
        HANDLE hFile = CreateFile(file.c_str(), GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL, NULL);
        if (hFile == INVALID_HANDLE_VALUE) {
            throw std::runtime_error{std::format("could not open file {}", file)};
        }

        // set sparse file: https://learn.microsoft.com/en-us/windows/win32/api/winioctl/ni-winioctl-fsctl_set_sparse
        DWORD bytesReturned;
        if (not DeviceIoControl(hFile, FSCTL_SET_SPARSE, NULL, 0, NULL, 0, &bytesReturned, NULL)) {
            CloseHandle(hFile);
            return SPARSE_STATUS::NO_SPARSE_FILE;
        }

        // close file handle
        CloseHandle(hFile);

        return SPARSE_STATUS::IS_SPARSE_FILE;
    }

    void grow_file(const std::string &file, const std::size_t size) {
        // open file
        HANDLE hFile = CreateFile(file.c_str(), GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL, NULL);
        if (hFile == INVALID_HANDLE_VALUE) {
            throw std::runtime_error{std::format("could not open file {}", file)};
        }

        // get file size: https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-getfilesizeex
        LARGE_INTEGER fileSize;
        GetFileSizeEx(hFile, &fileSize);

        // set file pointer: https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-setfilepointerex
        if (size > std::numeric_limits<LONGLONG>::max()) {
            throw std::runtime_error{
                    std::format("could not increase file {} to size {}, max limit reached", file, size)
            };
        }

        LARGE_INTEGER newSize;
        newSize.QuadPart = static_cast<LONGLONG>(size);
        if (not SetFilePointerEx(hFile, newSize, NULL, FILE_BEGIN)) {
            throw std::runtime_error{std::format("could not set file pointer in {} to position {}", file, size)};
            CloseHandle(hFile);
        }

        // set end of file: https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-setendoffile
        if (not SetEndOfFile(hFile)) {
            throw std::runtime_error{std::format("could grow file {} to size {}", file, size)};
            CloseHandle(hFile);
        }

        // set zero data: https://learn.microsoft.com/en-us/windows/win32/api/winioctl/ni-winioctl-fsctl_set_zero_data
        FILE_ZERO_DATA_INFORMATION zeroDataInfo;
        zeroDataInfo.FileOffset = fileSize;
        zeroDataInfo.BeyondFinalZero = newSize;

        DWORD bytesReturned;
        if (not DeviceIoControl(hFile, FSCTL_SET_ZERO_DATA, &zeroDataInfo, sizeof(zeroDataInfo), NULL, 0,
                &bytesReturned, NULL)) {
            CloseHandle(hFile);
        }

        // close file handle
        CloseHandle(hFile);
    }

    bool flush_address(void *addr, const std::size_t size) {
        // flush file view: https://learn.microsoft.com/en-us/windows/win32/api/memoryapi/nf-memoryapi-flushviewoffile
        return FlushViewOfFile(addr, size) == 0;
    }

    bool flush_file(const std::string &file) {
        // open file: https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilea
        HANDLE hFile = CreateFile(file.c_str(), GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL, NULL);
        if (hFile == INVALID_HANDLE_VALUE) {
            throw std::runtime_error{std::format("could not open file {}", file)};
        }

        // flush file buffers: https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-flushfilebuffers
        if (not FlushFileBuffers(hFile)) {
            throw std::runtime_error{std::format("could flush file {}", file)};
            CloseHandle(hFile);
        }

        // close file handle
        CloseHandle(hFile);
    }

#elif defined(MM2_LINUX)

    bool flush_address(void *addr, const std::size_t size, const bool async) {
        return msync(addr, size, async ? MS_ASYNC : MS_SYNC) == 0;
    }

#endif

}
