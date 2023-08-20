#include "common/IDebugLog.h"  // IDebugLog
#include "skse64_common/skse_version.h"  // RUNTIME_VERSION
#include "skse64/PluginAPI.h"  // SKSEInterface, PluginInfo
#include "skse64_common/BranchTrampoline.h"
#include "skse64/GameRTTI.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <ShlObj.h>  // CSIDL_MYDOCUMENTS

#include <d3d11.h>
#include <d3d11shader.h>
#include <d3dcompiler.h>
#include <winrt/base.h>
#include <wrl/client.h>

#include <Physics/Collide/Shape/Convex/Sphere/hkpSphereShape.h>
#include <Physics/Collide/Shape/Convex/Capsule/hkpCapsuleShape.h>
#include <Physics/Collide/Shape/Convex/ConvexVertices/hkpConvexVerticesShape.h>
#include <Physics/Collide/Shape/Convex/ConvexTransform/hkpConvexTransformShape.h>
#include <Physics/Collide/Shape/Misc/Transform/hkpTransformShape.h>
#include <Physics/Collide/Shape/Convex/Triangle/hkpTriangleShape.h>
#include <Physics/Dynamics/Phantom/hkpAabbPhantom.h>
#include <Physics/Dynamics/Constraint/Bilateral/Ragdoll/hkpRagdollConstraintData.h>

#include "config.h"
#include "havok.h"
#include "offsets.h"
#include "utils.h"
#include "version.h"

using Microsoft::WRL::ComPtr;


// SKSE globals
static PluginHandle	g_pluginHandle = kPluginHandle_Invalid;
static SKSEMessagingInterface *g_messaging = nullptr;

SKSETrampolineInterface *g_trampoline = nullptr;


struct RenderGlobals
{
    void *rendererData;					// RendererData *
    ID3D11Device *device; // 08
    HWND m_Window; // 10

    //
    // These are pools for efficient data uploads to the GPU. Each frame can use any buffer as long as there
    // is sufficient space. If there's no space left, delay execution until m_DynamicVertexBufferAvailQuery[] says a buffer
    // is no longer in use.
    //
    ID3D11Buffer *m_DynamicVertexBuffers[3];			// DYNAMIC (VERTEX | INDEX) CPU_ACCESS_WRITE
    UInt32			m_CurrentDynamicVertexBuffer;

    UInt32			m_CurrentDynamicVertexBufferOffset;	// Used in relation with m_DynamicVertexBufferAvailQuery[]
    ID3D11Buffer *m_SharedParticleIndexBuffer;		// DEFAULT INDEX CPU_ACCESS_NONE
    ID3D11Buffer *m_SharedParticleStaticBuffer;		// DEFAULT VERTEX CPU_ACCESS_NONE
    struct ID3D11InputLayout *m_ParticleShaderInputLayout;
    struct ID3D11InputLayout *m_UnknownInputLayout2;
    float oldClearColor[4]; // 58
    ID3D11DeviceContext *deviceContext; // 68
    UInt32 unk70;
    UInt32 unk74;
    struct ID3D11Query *queries[67]; // 78
    UInt32 unk290;
    UInt32 unk294;

    float m_DepthBiasFactors[3][4]; // 298
    UInt32 unk2C8;
    UInt32 unk2CC;

    struct ID3D11DepthStencilState *m_DepthStates[6][40];		// 2D0 - OMSetDepthStencilState
    struct ID3D11RasterizerState *m_RasterStates[2][3][13][2];	// A50 - RSSetState
    struct ID3D11BlendState *m_BlendStates[9][2][13][2];		// F30 - OMSetBlendState
    struct ID3D11SamplerState *m_SamplerStates[6][5];			// 1DD0 - Samplers[AddressMode][FilterMode] (Used for PS and CS)

    void *vertexShader1EC0; // 1EC0
    struct ID3D11InputLayout *inputLayoud1EC8; // 1EC8
    BSTriShape *hiddenAreaMesh; // 1ED0

    ID3D11Buffer *m_ConstantBuffers1[4];				// 1ED8 - Sizes: 3840 bytes
    ID3D11Buffer *m_AlphaTestRefCB;					// CONSTANT_GROUP_LEVEL_ALPHA_TEST_REF (Index 11) - 16 bytes
    ID3D11Buffer *m_ConstantBuffers2[20];			// Sizes: 0, 16, 32, 48, ... 304 bytes
    ID3D11Buffer *m_ConstantBuffers3[10];			// Sizes: 0, 16, 32, 48, ... 144 bytes
    ID3D11Buffer *m_ConstantBuffers4[42];			// Sizes: 0, 16, 32, 48, ... 432 bytes
    ID3D11Buffer *m_ConstantBuffers5[20];			// Sizes: 0, 16, 32, 48, ... 304 bytes
    ID3D11Buffer *m_ConstantBuffers6[20];			// Sizes: 0, 16, 32, 48, ... 304 bytes
    ID3D11Buffer *m_ConstantBuffers7[54];			// Sizes: 0, 16, 32, 48, ... 624 bytes
    ID3D11Buffer *m_TempConstantBuffer2;				// 576 bytes
    ID3D11Buffer *m_TempConstantBuffer3;				// CONSTANT_GROUP_LEVEL_PERFRAME (Index 12) - 720 bytes
    ID3D11Buffer *m_TempConstantBuffer4;				// 16 bytes

    struct IDXGIOutput *m_DXGIAdapterOutput; // 2448

    void *m_FrameDurationStringHandle;					// 2450 - "Frame Duration" but stored in their global string pool
};
RelocPtr<RenderGlobals> g_renderGlobals(0x317E790);

typedef __m128 XMVECTOR;

struct XMMATRIX
{
    // 0-2 rotation
    // 3 position
    XMVECTOR r[4];
};

typedef void(*_XMMATRIXFromNiTransform)(XMMATRIX *out, const NiTransform *transform, int eyeIndex);
RelocAddr<_XMMATRIXFromNiTransform> XMMATRIXFromNiTransform(0x1301660);

struct ViewData
{
    XMVECTOR m_ViewUp; // 00
    XMVECTOR m_ViewRight; // 10
    XMVECTOR m_ViewForward; // 20
    XMMATRIX m_ViewMat; // 30
    XMMATRIX m_ProjMat; // 70
    XMMATRIX m_ViewProjMat; // B0
    XMMATRIX m_UnknownMat1; // F0 - all 0?
    XMMATRIX m_ViewProjMatrixUnjittered; // 130
    XMMATRIX m_PreviousViewProjMatrixUnjittered; // 170
    XMMATRIX m_ProjMatrixUnjittered; // 1B0
    XMMATRIX m_UnknownMat2; // 1F0 - all 0?
    float m_ViewPort[4];// 230 - NiRect<float> { left = 0, right = 1, top = 1, bottom = 0 }
    NiPoint2 m_ViewDepthRange; // 240
    char _pad0[0x8]; // 248
}; // size == 250

struct RendererShadowState
{
    UInt32 m_StateUpdateFlags;						// Flags +0x0  0xFFFFFFFF; global state updates
    uint32_t m_PSResourceModifiedBits;					// Flags +0x4  0xFFFF
    uint32_t m_PSSamplerModifiedBits;					// Flags +0x8  0xFFFF
    uint32_t m_CSResourceModifiedBits;					// Flags +0xC  0xFFFF
    uint32_t m_CSSamplerModifiedBits;					// Flags +0x10 0xFFFF
    uint32_t m_CSUAVModifiedBits;						// Flags +0x14 0xFF
    uint32_t m_OMUAVModifiedBits;						// Flags +0x18 0xFF
    uint32_t m_SRVModifiedBits;						// Flags +0x1C 0xFF

    uint32_t m_RenderTargets[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT]; // 20
    uint32_t m_DepthStencil;							// 40 - Index
    uint32_t m_DepthStencilSlice;						// Index
    uint32_t m_CubeMapRenderTarget;						// 48 = Index
    uint32_t m_CubeMapRenderTargetView;					// Index

    UInt32 m_SetRenderTargetMode[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT]; // 50
    UInt32 m_SetDepthStencilMode; // 70
    UInt32 m_SetCubeMapRenderTargetMode;

    D3D11_VIEWPORT m_ViewPort; // 78

    UInt32 m_DepthStencilDepthMode;
    UInt32 m_DepthStencilDepthModePrevious; // 94 - also some kind of mode
    uint32_t m_DepthStencilStencilMode;
    uint32_t m_StencilRef;

    uint32_t m_RasterStateFillMode;
    uint32_t m_RasterStateCullMode;
    uint32_t m_RasterStateDepthBiasMode;
    uint32_t m_RasterStateScissorMode;

    uint32_t m_AlphaBlendMode;
    uint32_t m_AlphaBlendAlphaToCoverage;
    uint32_t m_AlphaBlendWriteMode;

    bool m_AlphaTestEnabled; // BC
    float m_AlphaTestRef; // C0

    uint32_t m_PSTextureAddressMode[16];
    uint32_t m_PSTextureFilterMode[16];
    ID3D11ShaderResourceView *m_PSTexture[16]; // 148

    uint32_t m_CSTextureAddressMode[16];
    uint32_t m_CSTextureFilterMode[16]; // 208

    ID3D11ShaderResourceView *m_CSTexture[16]; // 248
    uint32_t m_CSTextureMinLodMode[16]; // 2C8
    struct ID3D11UnorderedAccessView *m_CSUAV[8]; // 308

    UInt8 unk348[0x388 - 0x348]; // 348

    uint64_t m_VertexDesc; // 388
    void *m_CurrentVertexShader; // 390
    void *m_CurrentPixelShader; // 398
    D3D11_PRIMITIVE_TOPOLOGY m_Topology; // 3A0

    NiPoint3 m_PosAdjust[2]; // 3A4
    NiPoint3 m_PreviousPosAdjust[2]; // 3BC
    ViewData m_CameraData[2]; // 3E0 - size of each is 250

    uint32_t m_AlphaBlendModeExtra; // 880
    float unk884;
    float unk888;
    UInt32 unk88C;

    ID3D11Buffer *VSConstantBuffers[12]; // 890
    ID3D11Buffer *PSConstantBuffers[12]; // 8F0
};
RelocPtr<RendererShadowState> g_rendererShadowState(0x3180DB0);

RelocPtr<ID3D11Buffer *[]> g_currentVSConstantBuffers(0x3181640);
RelocPtr<ID3D11Buffer *[]> g_currentPSConstantBuffers(0x31816A0);


ID3D11Buffer *g_cameraBuffer = nullptr;
ID3D11Buffer *g_modelBuffer = nullptr;

struct ShapeBuffers
{
    ComPtr<ID3D11Buffer> vertexBuffer;
    ComPtr<ID3D11Buffer> indexBuffer;
    UINT numIndices = 0;
};

template <class T>
inline void hash_combine(std::size_t &seed, const T &v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct ShapeIdentifier
{
    // Why does this class exist?
    // It's because we can't just use shapes to index into the map, as the memory for a shape can get re-used at a later time, which would lead to incorrectly re-using the buffers for the wrong shape.
    // Using more object addresses to identify the shape makes it less likely to have address collisions.

    const bhkWorldObject *worldObjectWrapper;
    const hkpWorldObject *worldObject;
    const bhkShape *shapeWrapper;
    const hkpShape *shape;

    struct Hash
    {
        std::size_t operator()(const ShapeIdentifier &k) const
        {
            std::size_t h = 0;
            hash_combine(h, (UInt64)k.worldObjectWrapper);
            hash_combine(h, (UInt64)k.worldObject);
            hash_combine(h, (UInt64)k.shapeWrapper);
            hash_combine(h, (UInt64)k.shape);
            return h;
        }
    };

    struct Equal
    {
        bool operator()(const ShapeIdentifier &lhs, const ShapeIdentifier &rhs) const
        {
            return lhs.worldObjectWrapper == rhs.worldObjectWrapper && lhs.worldObject == rhs.worldObject && lhs.shapeWrapper == rhs.shapeWrapper && lhs.shape == rhs.shape;
        }
    };
};
std::unordered_map<ShapeIdentifier, ShapeBuffers, ShapeIdentifier::Hash, ShapeIdentifier::Equal> g_shapeBuffers{};

struct Vertex
{
    NiPoint3 pos;
};

ShapeBuffers CreateVertexAndIndexBuffers(std::vector<Vertex> &vertices, std::vector<WORD> &indices)
{
    ShapeBuffers buffers{};

    D3D11_BUFFER_DESC vertexBufferDesc;
    vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
    vertexBufferDesc.ByteWidth = sizeof(Vertex) * vertices.size();
    vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    vertexBufferDesc.CPUAccessFlags = 0;
    vertexBufferDesc.MiscFlags = 0;
    vertexBufferDesc.StructureByteStride = 0;

    D3D11_SUBRESOURCE_DATA vertexData;
    vertexData.pSysMem = vertices.data();
    vertexData.SysMemPitch = 0;
    vertexData.SysMemSlicePitch = 0;

    HRESULT result = g_renderGlobals->device->CreateBuffer(&vertexBufferDesc, &vertexData, buffers.vertexBuffer.GetAddressOf());
    if (FAILED(result)) {
        _ERROR("Failed to create vertex buffer");
        return buffers;
    }

    D3D11_BUFFER_DESC indexBufferDesc;
    indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
    indexBufferDesc.ByteWidth = sizeof(WORD) * indices.size();
    indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
    indexBufferDesc.CPUAccessFlags = 0;
    indexBufferDesc.MiscFlags = 0;
    indexBufferDesc.StructureByteStride = 0;

    D3D11_SUBRESOURCE_DATA indexData;
    indexData.pSysMem = indices.data();
    indexData.SysMemPitch = 0;
    indexData.SysMemSlicePitch = 0;

    result = g_renderGlobals->device->CreateBuffer(&indexBufferDesc, &indexData, buffers.indexBuffer.GetAddressOf());
    if (FAILED(result)) {
        _ERROR("Failed to create index buffer");
        return buffers;
    }

    buffers.numIndices = indices.size();

    return buffers;
}

struct PerFrameVSData
{
    XMMATRIX matProjView[2];
};

struct PerObjectVSData
{
    XMMATRIX matModel[2];
    NiColorA color;
};

void CreateConstantBuffers()
{
    // Create camera buffer
    D3D11_BUFFER_DESC cameraBufferDesc;
    cameraBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
    cameraBufferDesc.ByteWidth = sizeof(PerFrameVSData);
    cameraBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    cameraBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    cameraBufferDesc.MiscFlags = 0;
    cameraBufferDesc.StructureByteStride = 0;

    HRESULT result = g_renderGlobals->device->CreateBuffer(&cameraBufferDesc, nullptr, &g_cameraBuffer);
    if (FAILED(result)) {
        _ERROR("Failed to create camera buffer");
    }

    // Create model buffer
    D3D11_BUFFER_DESC modelBufferDesc;
    modelBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
    modelBufferDesc.ByteWidth = sizeof(PerObjectVSData);
    modelBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    modelBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    modelBufferDesc.MiscFlags = 0;
    modelBufferDesc.StructureByteStride = 0;

    result = g_renderGlobals->device->CreateBuffer(&modelBufferDesc, nullptr, &g_modelBuffer);
    if (FAILED(result)) {
        _ERROR("Failed to create model buffer");
    }
}

ID3D11RasterizerState *g_rasterizerState = nullptr;

void CreateRasterizerState()
{
    D3D11_RASTERIZER_DESC desc;
    if (Config::options.wireframe) {
        desc.FillMode = D3D11_FILL_WIREFRAME;
        desc.CullMode = D3D11_CULL_NONE;
    }
    else {
        desc.FillMode = D3D11_FILL_SOLID;
        desc.CullMode = D3D11_CULL_BACK;
    }
    desc.FrontCounterClockwise = true;
    desc.DepthBias = 0.f;
    desc.DepthBiasClamp = -100.f;
    desc.SlopeScaledDepthBias = 0.f;
    desc.DepthClipEnable = true;
    desc.ScissorEnable = false;
    desc.MultisampleEnable = false;
    desc.AntialiasedLineEnable = false;
    HRESULT result = g_renderGlobals->device->CreateRasterizerState(&desc, &g_rasterizerState);
    if (FAILED(result)) {
        _ERROR("Failed to create rasterizer state");
    }
}

ID3D11InputLayout *g_inputLayout = nullptr;
ID3D11VertexShader *g_vertexShader = nullptr;
ID3D11PixelShader *g_pixelShader = nullptr;

constexpr const auto vertexShaderSource = R"(
struct VS_INPUT {
    float3 vPos : POS;
    uint instanceId: SV_InstanceID0;
};

struct VS_OUTPUT {
    float4 vPos : SV_POSITION;
    float4 vColor : COLOR0;
    float clipDistance : SV_ClipDistance;
    float cullDistance : SV_CullDistance;
};

cbuffer Camera : register(b0) {
    float4x4 matProjView[2];
};

cbuffer Model : register(b1) {
    float4x4 matModel[2];
    float4 color;
};

VS_OUTPUT main(VS_INPUT input) {
    const float4 eyeClipEdge[2] = { { -1, 0, 0, 1 }, { 1, 0, 0, 1 } };
    const float eyeOffsetScale[2] = { -0.5, 0.5 };

    float4 pos = float4(input.vPos.xyz, 1.0f);
    pos = mul(matModel[input.instanceId], pos);
    pos = mul(matProjView[input.instanceId], pos);

    VS_OUTPUT output;
    output.vColor = color;
    output.clipDistance = mul(pos, eyeClipEdge[input.instanceId]);
    output.cullDistance = output.clipDistance;

    pos.x *= 0.5;
    pos.x += eyeOffsetScale[input.instanceId] * pos.w;
    output.vPos = pos;

    return output;
}
        )";

constexpr const auto pixelShaderSource = R"(
struct PS_INPUT {
    float4 pos : SV_POSITION;
    float4 color : COLOR0;
};

struct PS_OUTPUT {
    float4 color : SV_Target;
};

PS_OUTPUT main(PS_INPUT input) {
    PS_OUTPUT output;
    output.color = input.color;
    return output;
}
        )";

void CreateShaders()
{
    // Create vertex shader

    UINT compileFlags = D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR;
    winrt::com_ptr<ID3DBlob> errorBlob;
    winrt::com_ptr<ID3DBlob> vertexShaderBinary;

    HRESULT result = D3DCompile(
        vertexShaderSource,
        strlen(vertexShaderSource),
        nullptr,
        nullptr,
        nullptr,
        "main",
        "vs_5_0",
        compileFlags,
        0,
        vertexShaderBinary.put(),
        errorBlob.put()
    );

    if (FAILED(result)) {
        _ERROR("Vertex shader failed to compile");
        if (errorBlob) {
            _ERROR(static_cast<LPCSTR>(errorBlob->GetBufferPointer()));
        }
    }

    result = g_renderGlobals->device->CreateVertexShader(vertexShaderBinary->GetBufferPointer(), vertexShaderBinary->GetBufferSize(), nullptr, &g_vertexShader);
    if (FAILED(result)) {
        _ERROR("Failed to create vertex shader");
    }

    // Create pixel shader

    winrt::com_ptr<ID3DBlob> pixelShaderBinary;

    result = D3DCompile(
        pixelShaderSource,
        strlen(pixelShaderSource),
        nullptr,
        nullptr,
        nullptr,
        "main",
        "ps_5_0",
        compileFlags,
        0,
        pixelShaderBinary.put(),
        errorBlob.put()
    );

    if (FAILED(result)) {
        if (errorBlob) {
            _ERROR(static_cast<LPCSTR>(errorBlob->GetBufferPointer()));
        }
    }

    result = g_renderGlobals->device->CreatePixelShader(pixelShaderBinary->GetBufferPointer(), pixelShaderBinary->GetBufferSize(), nullptr, &g_pixelShader);
    if (FAILED(result)) {
        _ERROR("Failed to create pixel shader");
    }

    // Create input layout
    std::vector<D3D11_INPUT_ELEMENT_DESC> layout = {
        { "POS", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
    };

    result = g_renderGlobals->device->CreateInputLayout(
        layout.data(), layout.size(),
        vertexShaderBinary->GetBufferPointer(), vertexShaderBinary->GetBufferSize(),
        &g_inputLayout
    );
    if (FAILED(result)) {
        _ERROR("Failed to create input layout");
    }
}

void SetVSConstantBuffer(UInt32 startSlot, ID3D11Buffer *buffer)
{
    // We do this so that the game properly sets the constant buffer next time it checks if it needs to
    ID3D11Buffer *&currentBuffer = (*g_currentVSConstantBuffers)[startSlot];
    if (buffer != currentBuffer) {
        g_renderGlobals->deviceContext->VSSetConstantBuffers(startSlot, 1, &buffer);
        currentBuffer = buffer;
    }
}

void SetPSConstantBuffer(UInt32 startSlot, ID3D11Buffer *buffer)
{
    ID3D11Buffer *&currentBuffer = (*g_currentPSConstantBuffers)[startSlot];
    if (buffer != currentBuffer) {
        g_renderGlobals->deviceContext->PSSetConstantBuffers(startSlot, 1, &buffer);
        currentBuffer = buffer;
    }
}

std::pair<std::vector<Vertex>, std::vector<WORD>> GetTriangleVertices(hkpTriangleShape *shape)
{
    std::vector<Vertex> vertices{
        { HkVectorToNiPoint(shape->getVertex(0)) },
        { HkVectorToNiPoint(shape->getVertex(1)) },
        { HkVectorToNiPoint(shape->getVertex(2)) }
    };

    std::vector<WORD> indices = { 0, 1, 2 };

    return { vertices, indices };
}

std::pair<std::vector<Vertex>, std::vector<WORD>> GetTriangleListVertices(const hkpShapeContainer *container)
{
    std::vector<Vertex> vertices{};
    std::vector<WORD> indices{};

    hkpShapeBuffer buffer{};

    int numTriangles = 0;

    hkpShapeKey shapeKey = container->getFirstKey();
    while (shapeKey != HK_INVALID_SHAPE_KEY) {
        if (const hkpShape *childShape = container->getChildShape(shapeKey, buffer)) {
            if (childShape->getType() == hkpShapeType::HK_SHAPE_TRIANGLE) {
                hkpTriangleShape *triangleShape = DYNAMIC_CAST(childShape, hkpShape, hkpTriangleShape);

                vertices.push_back({ HkVectorToNiPoint(triangleShape->getVertex(0)) * *g_inverseHavokWorldScale });
                vertices.push_back({ HkVectorToNiPoint(triangleShape->getVertex(1)) * *g_inverseHavokWorldScale });
                vertices.push_back({ HkVectorToNiPoint(triangleShape->getVertex(2)) * *g_inverseHavokWorldScale });

                int offset = numTriangles * 3;
                indices.push_back(offset);
                indices.push_back(offset + 1);
                indices.push_back(offset + 2);

                numTriangles++;
            }
        }
        shapeKey = container->getNextKey(shapeKey);
    }

    return { vertices, indices };
}

std::pair<std::vector<Vertex>, std::vector<WORD>> GetSphereVertices(float radius)
{
    radius *= *g_inverseHavokWorldScale;

    std::vector<Vertex> vertices;
    std::vector<WORD> indices;

    int numSegments = Config::options.numSphereSegments;
    int numRings = numSegments;

    for (int ring = 0; ring <= numRings; ring++) {
        const float v = ring / (float)numRings;
        const float theta = v * M_PI;

        for (int segment = 0; segment <= numSegments; segment++) {
            const float u = segment / (float)numSegments;
            const float phi = u * M_PI * 2;

            const float x = cos(phi) * sin(theta);
            const float y = cos(theta);
            const float z = sin(phi) * sin(theta);

            Vertex vertex;
            vertex.pos = { x * radius, y * radius, z * radius };
            vertices.push_back(vertex);
        }
    }

    for (int ring = 0; ring < numRings; ring++) {
        for (int segment = 0; segment < numSegments; segment++) {
            const int index = ring * (numSegments + 1) + segment;

            indices.push_back(index);
            indices.push_back(index + numSegments + 1);
            indices.push_back(index + 1);

            indices.push_back(index + 1);
            indices.push_back(index + numSegments + 1);
            indices.push_back(index + numSegments + 2);
        }
    }

    return { vertices, indices };
}

std::pair<std::vector<Vertex>, std::vector<WORD>> GetCapsuleVertices(hkVector4 &a_vertexA, hkVector4 &a_vertexB, float a_radius)
{
    // Create vertices and indices for a capsule shape with the given radius and endpoints

    int numSegments = Config::options.numCapsuleSegments;

    const int numRings = numSegments / 2; // Each cap is half a sphere, so use half the number of segments

    float radius = a_radius * *g_inverseHavokWorldScale;

    NiPoint3 vertexA = HkVectorToNiPoint(a_vertexA) * *g_inverseHavokWorldScale;
    NiPoint3 vertexB = HkVectorToNiPoint(a_vertexB) * *g_inverseHavokWorldScale;

    NiPoint3 direction = vertexB - vertexA;
    float length = VectorLength(direction);
    direction /= length;

    if (length < 0.001f) {
        // If the capsule is too short, just draw a sphere
        return GetSphereVertices(radius);
    }

    NiPoint3 perpendicular = { 0.f, 0.f, 1.f };
    if (fabsf(DotProduct(direction, perpendicular)) > 0.999f) {
        perpendicular = { 1.f, 0.f, 0.f };
    }

    NiPoint3 axis = CrossProduct(direction, perpendicular);
    axis = VectorNormalized(axis);

    perpendicular = CrossProduct(axis, direction);
    perpendicular = VectorNormalized(perpendicular);

    std::vector<Vertex> cylinderVertices;
    std::vector<WORD> cylinderIndices;

    {
        // Create the vertices for the cylinder part
        for (int segment = 0; segment <= numSegments; segment++) {
            const float theta = segment / (float)numSegments * M_PI * 2;

            for (int i = 0; i < 2; i++) {
                Vertex vertex;
                vertex.pos = vertexA + direction * (i * length) + axis * (cos(theta) * radius) + perpendicular * (sin(theta) * radius);
                cylinderVertices.push_back(vertex);
            }
        }

        // Create the indices for the cylinder part
        for (int segment = 0; segment < numSegments; segment++) {
            const int index = segment * 2;

            cylinderIndices.push_back(index);
            cylinderIndices.push_back(index + 1);
            cylinderIndices.push_back(index + 2);

            cylinderIndices.push_back(index + 2);
            cylinderIndices.push_back(index + 1);
            cylinderIndices.push_back(index + 3);
        }
    }

    std::vector<Vertex> sphereVertices;
    std::vector<WORD> sphereIndices;

    {
        // Create the vertices for the first sphere cap

        int numVerts = cylinderVertices.size();

        for (int ring = 0; ring <= numRings; ring++) {
            const float v = ring / (float)numRings;
            const float theta = v * M_PI / 2;

            for (int segment = 0; segment <= numSegments; segment++) {
                const float u = segment / (float)numSegments;
                const float phi = u * M_PI * 2;

                const float x = cos(phi) * sin(theta);
                const float y = cos(theta);
                const float z = sin(phi) * sin(theta);

                NiPoint3 pos = axis * x + direction * y + perpendicular * z;

                Vertex vertex;
                vertex.pos = pos * radius;
                vertex.pos += vertexB;
                sphereVertices.push_back(vertex);
            }
        }

        // Create the indices for the first sphere cap

        for (int ring = 0; ring < numRings; ring++) {
            for (int segment = 0; segment < numSegments; segment++) {
                int index = ring * (numSegments + 1) + segment;
                index += numVerts;

                sphereIndices.push_back(index + 1);
                sphereIndices.push_back(index + numSegments + 1);
                sphereIndices.push_back(index);

                sphereIndices.push_back(index + numSegments + 2);
                sphereIndices.push_back(index + numSegments + 1);
                sphereIndices.push_back(index + 1);
            }
        }

        numVerts = cylinderVertices.size() + sphereVertices.size();

        for (int ring = 0; ring <= numRings; ring++) {
            const float v = ring / (float)numRings;
            const float theta = (M_PI / 2) + (v * M_PI / 2);

            for (int segment = 0; segment <= numSegments; segment++) {
                const float u = segment / (float)numSegments;
                const float phi = u * M_PI * 2;

                const float x = cos(phi) * sin(theta);
                const float y = cos(theta);
                const float z = sin(phi) * sin(theta);

                NiPoint3 pos = axis * x + direction * y + perpendicular * z;

                Vertex vertex;
                vertex.pos = pos * radius;
                vertex.pos += vertexA;
                sphereVertices.push_back(vertex);
            }
        }

        for (int ring = 0; ring < numRings; ring++) {
            for (int segment = 0; segment < numSegments; segment++) {
                int index = ring * (numSegments + 1) + segment;
                index += numVerts;

                sphereIndices.push_back(index + 1);
                sphereIndices.push_back(index + numSegments + 1);
                sphereIndices.push_back(index);

                sphereIndices.push_back(index + numSegments + 2);
                sphereIndices.push_back(index + numSegments + 1);
                sphereIndices.push_back(index + 1);
            }
        }
    }

    std::vector<Vertex> vertices;
    std::vector<WORD> indices;

    vertices.insert(vertices.end(), cylinderVertices.begin(), cylinderVertices.end());
    vertices.insert(vertices.end(), sphereVertices.begin(), sphereVertices.end());

    indices.insert(indices.end(), cylinderIndices.begin(), cylinderIndices.end());
    indices.insert(indices.end(), sphereIndices.begin(), sphereIndices.end());

    return { vertices, indices };
}

std::vector<std::tuple<int, int, int>> GenerateTrianglesForConvexHull(const std::vector<Vertex> &a_vertices)
{
    int numVerts = a_vertices.size();

    std::vector<NiPoint3> vertices(numVerts);
    std::vector<std::tuple<int, int, int>> tris;

    for (int v = 0; v < numVerts; v++) {
        vertices[v] = a_vertices[v].pos * *g_inverseHavokWorldScale;
    }

    // First let's compute the centroid to compare normals against
    NiPoint3 centroid(0, 0, 0);
    for (int v = 0; v < numVerts; v++) {
        centroid += vertices[v];
    }
    centroid /= numVerts;

    // Go through every potential triangle (set of 3 verts)
    for (int i = 0; i < numVerts - 2; i++) {
        NiPoint3 vertA = vertices[i];

        for (int j = i + 1; j < numVerts - 1; j++) {
            NiPoint3 vertB = vertices[j];

            for (int k = j + 1; k < numVerts; k++) {
                NiPoint3 vertC = vertices[k];

                NiPoint3 triNormal = CrossProduct((vertB - vertA), (vertC - vertA));

                // Check if all other vertices are on the same side of the triangle
                bool isHullTriangle = true;
                int prevSide = 0;
                for (int p = 0; p < numVerts; p++) {
                    NiPoint3 testVert = vertices[p];

                    if (p == i || p == j || p == k) continue;

                    float d = DotProduct((testVert - vertA), triNormal);
                    if (abs(d) < 0.001f) continue;

                    int side = d > 0.f ? 1 : -1;

                    if (side + prevSide == 0) {
                        // opposite sides
                        isHullTriangle = false;
                        break;
                    }

                    prevSide = side;
                }

                if (isHullTriangle) {
                    // Make sure the triangle is winded properly
                    if (DotProduct(triNormal, vertA - centroid) < 0.f) {
                        tris.push_back({ k, j, i });
                    }
                    else {
                        tris.push_back({ i, j, k });
                    }
                }
            }
        }
    }

    return tris;
}

bool AreVerticesInTheSamePlane(std::vector<Vertex> &vertices)
{
    if (vertices.size() <= 3) return false;

    NiPoint3 normal = VectorNormalized(CrossProduct(vertices[1].pos - vertices[0].pos, vertices[2].pos - vertices[0].pos));

    for (int i = 3; i < vertices.size(); i++) {
        NiPoint3 testNormal = VectorNormalized(CrossProduct(vertices[i].pos - vertices[0].pos, vertices[i].pos - vertices[1].pos));

        if (abs(DotProduct(normal, testNormal)) < 0.999f) {
            return false;
        }
    }

    return true;
}

std::vector<std::tuple<int, int, int>> TriangulateConvexVertices(std::vector<Vertex> &vertices, float convexRadius)
{
    // Generate the initial convex hull
    std::vector<std::tuple<int, int, int>> triangles = GenerateTrianglesForConvexHull(vertices);

    if (Config::options.inflateByConvexRadius && convexRadius > 0.f) {
        std::vector<Vertex> newVertices{};

        // Extrude a new vertex for each triangle touching each original vertex
        for (int i = 0; i < vertices.size(); i++) {
            std::vector<std::tuple<int, int, int>> adjacentTriangles{};
            for (auto &triangle : triangles) {
                auto [p, j, k] = triangle;
                if (p == i || j == i || k == i) {
                    adjacentTriangles.push_back(triangle);
                }
            }

            for (auto &triangle : adjacentTriangles) {
                auto [p, j, k] = triangle;

                NiPoint3 A = vertices[p].pos;
                NiPoint3 B = vertices[j].pos;
                NiPoint3 C = vertices[k].pos;

                NiPoint3 normal = VectorNormalized(CrossProduct(B - A, C - A));
                NiPoint3 newVert = vertices[i].pos + normal * convexRadius * *g_inverseHavokWorldScale;

                newVertices.push_back({ newVert });
            }
        }

        { // Also extrude a new vertex in the direction of the vertex normal
            std::vector<NiPoint3> normals(vertices.size());
            for (auto &triangle : triangles) {
                auto [i, j, k] = triangle;

                NiPoint3 A = vertices[i].pos;
                NiPoint3 B = vertices[j].pos;
                NiPoint3 C = vertices[k].pos;

                NiPoint3 unnormalizedNormal = CrossProduct(B - A, C - A);
                normals[i] += unnormalizedNormal;
                normals[j] += unnormalizedNormal;
                normals[k] += unnormalizedNormal;
            }

            for (int i = 0; i < vertices.size(); i++) {
                Vertex &vertex = vertices[i];
                NiPoint3 normal = VectorNormalized(normals[i]);
                NiPoint3 newVert = vertex.pos + normal * convexRadius * *g_inverseHavokWorldScale;

                newVertices.push_back({ newVert });
            }
        }

        // Do another deduplication pass
        vertices = {};
        for (Vertex &vertex : newVertices) {
            bool isDuplicate = false;
            if (Config::options.dedupConvexVertices) {
                // This is kind of slow
                for (Vertex &other : vertices) {
                    if (VectorLengthSquared(other.pos - vertex.pos) < Config::options.dedupConvexVerticesThresholdCleanup) {
                        isDuplicate = true;
                        break;
                    }
                }
            }

            if (!isDuplicate) {
                vertices.push_back(vertex);
            }
        }

        // Re-generate the convex hull with the new vertices
        triangles = GenerateTrianglesForConvexHull(vertices);
    }

    return triangles;
}

std::pair<std::vector<Vertex>, std::vector<WORD>> GetConvexVerticesShapeVertices(hkpConvexVerticesShape *shape)
{
    static hkArray<hkVector4> g_scratchHkArray{};
    g_scratchHkArray.clear();
    hkArray<hkVector4> &verts = g_scratchHkArray;
    hkpConvexVerticesShape_getOriginalVertices(shape, verts);

    std::vector<Vertex> vertices{};
    std::vector<WORD> indices{};

    // TODO: Do multiple iterations of the convex radius inflation to get a better approximation of the shape, if the convex radius is large enough
    // TODO: Don't do the convex radius inflation if the convex radius is small enough

    // First, create the vertex buffer from the given verts
    for (hkVector4 &vert : verts) {
        Vertex vertex;
        vertex.pos = HkVectorToNiPoint(vert) * *g_inverseHavokWorldScale;

        bool isDuplicate = false;
        if (Config::options.dedupConvexVertices) {
            // This is kind of slow
            for (Vertex &other : vertices) {
                if (VectorLengthSquared(other.pos - vertex.pos) < Config::options.dedupConvexVerticesThreshold) {
                    isDuplicate = true;
                    break;
                }
            }
        }

        if (!isDuplicate) {
            vertices.push_back(vertex);
        }
    }

    if (Config::options.duplicatePlanarShapeVertices && AreVerticesInTheSamePlane(vertices)) {
        // Create duplicate vertices that are slightly offset, in the direction of the plane
        NiPoint3 normal = VectorNormalized(CrossProduct(vertices[1].pos - vertices[0].pos, vertices[2].pos - vertices[0].pos));
        int numVerts = vertices.size(); // get the size before we start adding new vertices
        for (int i = 0; i < numVerts; i++) {
            vertices.push_back({ vertices[i].pos + normal * 0.01f });
        }
    }

    // Now, we need to figure out which vertices to join into triangles for this convex shape
    std::vector<std::tuple<int, int, int>> triangles = TriangulateConvexVertices(vertices, shape->getRadius());

    for (auto &triangle : triangles) {
        auto [vert0, vert1, vert2] = triangle;
        indices.push_back(vert0);
        indices.push_back(vert1);
        indices.push_back(vert2);
    }

    return { vertices, indices };
}

std::pair<std::vector<Vertex>, std::vector<WORD>> GetBoxVertices(hkVector4 &a_halfExtents, float convexRadius, std::optional<NiPoint3> a_center = std::nullopt)
{
    NiPoint3 halfExtents = HkVectorToNiPoint(a_halfExtents);

    std::vector<Vertex> vertices = {
            {{-1.f, -1.f, -1.f}},
            {{-1.f, -1.f,  1.f}},
            {{-1.f,  1.f, -1.f}},
            {{-1.f,  1.f,  1.f}},
            {{ 1.f, -1.f, -1.f}},
            {{ 1.f, -1.f,  1.f}},
            {{ 1.f,  1.f, -1.f}},
            {{ 1.f,  1.f,  1.f}}
    };

    for (Vertex &vertex : vertices) {
        vertex.pos.x *= halfExtents.x * *g_inverseHavokWorldScale;
        vertex.pos.y *= halfExtents.y * *g_inverseHavokWorldScale;
        vertex.pos.z *= halfExtents.z * *g_inverseHavokWorldScale;

        if (a_center) {
            vertex.pos += *a_center * *g_inverseHavokWorldScale;
        }
    }

    std::vector<std::tuple<int, int, int>> triangles = TriangulateConvexVertices(vertices, convexRadius);

    std::vector<WORD> indices{};
    for (auto &triangle : triangles) {
        auto [vert0, vert1, vert2] = triangle;
        indices.push_back(vert0);
        indices.push_back(vert1);
        indices.push_back(vert2);
    }

    return { vertices, indices };
}

bool IsListShape(const hkpShape *shape)
{
    return shape->getType() == hkpShapeType::HK_SHAPE_LIST || shape->getType() == hkpShapeType::HK_SHAPE_CONVEX_LIST || shape->getType() == hkpShapeType::HK_SHAPE_MOPP;
}

void DrawShape(const ShapeIdentifier &shapeIdentifier, const hkpShape *shape, const NiTransform &transform, const NiColorA &color)
{
    if (IsListShape(shape)) {
        const hkpShapeContainer *container = shape->getContainer();
        if (!container) return;

        hkpShapeBuffer buffer{};

        hkpShapeKey shapeKey = container->getFirstKey();
        while (shapeKey != HK_INVALID_SHAPE_KEY) {
            if (const hkpShape *childShape = container->getChildShape(shapeKey, buffer)) {
                if (childShape->getType() != hkpShapeType::HK_SHAPE_TRIANGLE) {
                    ShapeIdentifier childShapeIdentifier = shapeIdentifier;
                    childShapeIdentifier.shape = childShape;
                    childShapeIdentifier.shapeWrapper = childShape->m_userData ? (bhkShape *)childShape->m_userData : nullptr;

                    DrawShape(childShapeIdentifier, childShape, transform, color);
                }
            }
            shapeKey = container->getNextKey(shapeKey);
        }

        // Don't return early here, as these shape collections may have their triangle child shapes handled later
    }
    else if (shape->getType() == hkpShapeType::HK_SHAPE_CONVEX_TRANSFORM) {
        hkpConvexTransformShape *transformShape = DYNAMIC_CAST(shape, hkpShape, hkpConvexTransformShape);
        if (!transformShape) return;

        NiTransform childTransform = transform * hkTransformToNiTransform(transformShape->getTransform());

        ShapeIdentifier childShapeIdentifier = shapeIdentifier;
        childShapeIdentifier.shape = transformShape->getChildShape();
        childShapeIdentifier.shapeWrapper = transformShape->getChildShape()->m_userData ? (bhkShape *)transformShape->getChildShape()->m_userData : nullptr;

        DrawShape(childShapeIdentifier, childShapeIdentifier.shape, childTransform, color);
        return;
    }
    else if (shape->getType() == hkpShapeType::HK_SHAPE_TRANSFORM) {
        hkpTransformShape *transformShape = DYNAMIC_CAST(shape, hkpShape, hkpTransformShape);
        if (!transformShape) return;

        NiTransform childTransform = transform * hkTransformToNiTransform(transformShape->getTransform());

        ShapeIdentifier childShapeIdentifier = shapeIdentifier;
        childShapeIdentifier.shape = transformShape->getChildShape();
        childShapeIdentifier.shapeWrapper = transformShape->getChildShape()->m_userData ? (bhkShape *)transformShape->getChildShape()->m_userData : nullptr;

        DrawShape(childShapeIdentifier, childShapeIdentifier.shape, childTransform, color);
        return;
    }

    // If we've already created the vertex and index buffers for this shape, use them. Otherwise, create them.
    auto &it = g_shapeBuffers.find(shapeIdentifier);
    if (it == g_shapeBuffers.end()) {
        if (IsListShape(shape)) {
            const hkpShapeContainer *container = shape->getContainer();
            if (!container) return;

            auto [vertices, indices] = GetTriangleListVertices(container);
            if (vertices.size() > 0) {
                g_shapeBuffers[shapeIdentifier] = CreateVertexAndIndexBuffers(vertices, indices);
            }
            else {
                g_shapeBuffers[shapeIdentifier] = { nullptr, nullptr, 0 };
            }
        }
        else if (shape->getType() == hkpShapeType::HK_SHAPE_BOX) {
            hkpBoxShape *boxShape = DYNAMIC_CAST(shape, hkpShape, hkpBoxShape);
            if (!boxShape) return;

            hkVector4 halfExtents = boxShape->getHalfExtents();
            float radius = boxShape->getRadius();
            auto [vertices, indices] = GetBoxVertices(halfExtents, radius);
            g_shapeBuffers[shapeIdentifier] = CreateVertexAndIndexBuffers(vertices, indices);
        }
        else if (shape->getType() == hkpShapeType::HK_SHAPE_SPHERE) {
            hkpSphereShape *sphereShape = DYNAMIC_CAST(shape, hkpShape, hkpSphereShape);
            if (!sphereShape) return;

            float radius = sphereShape->getRadius();
            auto [vertices, indices] = GetSphereVertices(radius);
            g_shapeBuffers[shapeIdentifier] = CreateVertexAndIndexBuffers(vertices, indices);
        }
        else if (shape->getType() == hkpShapeType::HK_SHAPE_CAPSULE) {
            hkpCapsuleShape *capsuleShape = DYNAMIC_CAST(shape, hkpShape, hkpCapsuleShape);
            if (!capsuleShape) return;

            hkVector4 vertexA = capsuleShape->getVertex(0);
            hkVector4 vertexB = capsuleShape->getVertex(1);
            float radius = capsuleShape->getRadius();
            auto [vertices, indices] = GetCapsuleVertices(vertexA, vertexB, radius);
            g_shapeBuffers[shapeIdentifier] = CreateVertexAndIndexBuffers(vertices, indices);
        }
        else if (shape->getType() == hkpShapeType::HK_SHAPE_CONVEX_VERTICES) {
            hkpConvexVerticesShape *convexVerticesShape = DYNAMIC_CAST(shape, hkpShape, hkpConvexVerticesShape);
            if (!convexVerticesShape) return;

            auto [vertices, indices] = GetConvexVerticesShapeVertices(convexVerticesShape);
            g_shapeBuffers[shapeIdentifier] = CreateVertexAndIndexBuffers(vertices, indices);
        }
        else {
            // TODO:
            // HK_SHAPE_BV_TREE, // 8 - terrain?
            // HK_SHAPE_BV, // 1C - terrain?

            return;
        }

        it = g_shapeBuffers.find(shapeIdentifier);
    }

    if (it->second.numIndices == 0) return;

    UINT stride = sizeof(Vertex);
    UINT offset = 0;
    g_renderGlobals->deviceContext->IASetVertexBuffers(0, 1, it->second.vertexBuffer.GetAddressOf(), &stride, &offset);
    g_renderGlobals->deviceContext->IASetIndexBuffer(it->second.indexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);

    { // Model data (object transform)

        // Each eye
        PerObjectVSData modelData;
        XMMATRIXFromNiTransform(&modelData.matModel[0], &transform, 0);
        XMMATRIXFromNiTransform(&modelData.matModel[1], &transform, 1);
        modelData.color = color;

        D3D11_MAPPED_SUBRESOURCE mappedResource;
        g_renderGlobals->deviceContext->Map(g_modelBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        memcpy(mappedResource.pData, &modelData, sizeof(PerObjectVSData));
        g_renderGlobals->deviceContext->Unmap(g_modelBuffer, 0);

        SetVSConstantBuffer(1, g_modelBuffer);
    }

    g_renderGlobals->deviceContext->DrawIndexedInstanced(it->second.numIndices, 2, 0, 0, 0);
}

ShapeBuffers g_pivotSphereBuffers;

void DrawConstraint(hkpConstraintInstance *constraint)
{
    hkpConstraintData *constraintData = constraint->m_data;
    if (!constraintData) return;

    if (constraintData->getType() == hkpConstraintData::ConstraintType::CONSTRAINT_TYPE_RAGDOLL) {
        hkpRagdollConstraintData *ragdollConstraintData = DYNAMIC_CAST(constraintData, hkpConstraintData, hkpRagdollConstraintData);
        if (!ragdollConstraintData) return;

        NiTransform pivotABodySpace = hkTransformToNiTransform(ragdollConstraintData->m_atoms.m_transforms.m_transformA);
        hkpEntity *entityA = constraint->getEntityA();
        hkpRigidBody *rigidBodyA = DYNAMIC_CAST(entityA, hkpEntity, hkpRigidBody);
        NiTransform entityATransform = hkTransformToNiTransform(rigidBodyA->getTransform());
        NiTransform transform = entityATransform * pivotABodySpace;

        UINT stride = sizeof(Vertex);
        UINT offset = 0;
        g_renderGlobals->deviceContext->IASetVertexBuffers(0, 1, g_pivotSphereBuffers.vertexBuffer.GetAddressOf(), &stride, &offset);
        g_renderGlobals->deviceContext->IASetIndexBuffer(g_pivotSphereBuffers.indexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);

        NiColorA color = { 0.f, 0.f, 1.f, 1.f };

        { // Model data (object transform)

            // Each eye
            PerObjectVSData modelData;
            XMMATRIXFromNiTransform(&modelData.matModel[0], &transform, 0);
            XMMATRIXFromNiTransform(&modelData.matModel[1], &transform, 1);
            modelData.color = color;

            D3D11_MAPPED_SUBRESOURCE mappedResource;
            g_renderGlobals->deviceContext->Map(g_modelBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
            memcpy(mappedResource.pData, &modelData, sizeof(PerObjectVSData));
            g_renderGlobals->deviceContext->Unmap(g_modelBuffer, 0);

            SetVSConstantBuffer(1, g_modelBuffer);
        }

        g_renderGlobals->deviceContext->DrawIndexedInstanced(g_pivotSphereBuffers.numIndices, 2, 0, 0, 0);
    }
}

void DrawRigidBody(const hkpRigidBody *rigidBody, float drawDistance)
{
    bhkRigidBody *wrapper = (bhkRigidBody *)rigidBody->m_userData;
    if (!wrapper) return;

    UInt32 filterInfo = rigidBody->getCollisionFilterInfo();
    if (filterInfo >> 14 & 1) return; // collision is disabled

    UInt32 layer = GetCollisionLayer(filterInfo);
    if (Config::options.ignoreLayers.count(layer)) return;

    const hkpShape *shape = rigidBody->getCollidable()->getShape();
    if (!shape) return;

    bhkShape *shapeWrapper = (bhkShape *)shape->m_userData;
    if (!shapeWrapper) return;

    const hkTransform &transform = rigidBody->getTransform();

    {
        hkAabb aabb; shape->getAabb(transform, 0.001f, aabb);
        aabb.expandBy(drawDistance);
        if (!aabb.containsPoint(NiPointToHkVector((*g_thePlayer)->pos * *g_havokWorldScale))) return;
    }

    NiColorA color = Config::options.defaultColor;

    {
        auto it = Config::options.layerColors.find(layer);
        if (it != Config::options.layerColors.end()) {
            color = it->second;
        }
    }

    hkpMotion::MotionType motionType = rigidBody->getMotionType();
    if (motionType == hkpMotion::MOTION_FIXED) {
        color.r *= Config::options.fixedObjectDimFactor;
        color.g *= Config::options.fixedObjectDimFactor;
        color.b *= Config::options.fixedObjectDimFactor;
    }

    ShapeIdentifier shapeIdentifier = { wrapper, rigidBody, shapeWrapper, shape };
    DrawShape(shapeIdentifier, shape, hkTransformToNiTransform(transform), color);

    for (hkpConstraintInstance *constraint : rigidBody->getConstraintSlaves()) {
        DrawConstraint(constraint);
    }
}

void DrawAabb(const ShapeIdentifier &shapeIdentifier, const hkAabb &aabb, const NiColorA &color)
{
    auto &it = g_shapeBuffers.find(shapeIdentifier);
    if (it == g_shapeBuffers.end()) {
        // First compute the half extents
        hkVector4 halfExtents;
        hkVector4 sub; sub.setSub4(aabb.m_max, aabb.m_min);
        halfExtents.setMul4({ 0.5f, 0.5f, 0.5f, 0.5f }, sub);

        // Now compute the center
        hkVector4 center; center.setAdd4(aabb.m_min, halfExtents);

        auto [vertices, indices] = GetBoxVertices(halfExtents, 0.f, HkVectorToNiPoint(center));
        g_shapeBuffers[shapeIdentifier] = CreateVertexAndIndexBuffers(vertices, indices);

        it = g_shapeBuffers.find(shapeIdentifier);
    }

    if (it->second.numIndices == 0) return;

    UINT stride = sizeof(Vertex);
    UINT offset = 0;
    g_renderGlobals->deviceContext->IASetVertexBuffers(0, 1, it->second.vertexBuffer.GetAddressOf(), &stride, &offset);
    g_renderGlobals->deviceContext->IASetIndexBuffer(it->second.indexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);

    { // Model data (object transform)

        // Each eye
        NiTransform transform = NiTransform(); // identity
        PerObjectVSData modelData;
        XMMATRIXFromNiTransform(&modelData.matModel[0], &transform, 0);
        XMMATRIXFromNiTransform(&modelData.matModel[1], &transform, 1);
        modelData.color = color;

        D3D11_MAPPED_SUBRESOURCE mappedResource;
        g_renderGlobals->deviceContext->Map(g_modelBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        memcpy(mappedResource.pData, &modelData, sizeof(PerObjectVSData));
        g_renderGlobals->deviceContext->Unmap(g_modelBuffer, 0);

        SetVSConstantBuffer(1, g_modelBuffer);
    }

    g_renderGlobals->deviceContext->DrawIndexedInstanced(it->second.numIndices, 2, 0, 0, 0);
}

void DrawPhantom(hkpPhantom *phantom, float drawDistance)
{
    bhkWorldObject *wrapper = (bhkWorldObject *)phantom->m_userData;
    if (!wrapper) return;

    UInt32 filterInfo = phantom->getCollidable()->getCollisionFilterInfo();
    if (filterInfo >> 14 & 1) return; // collision is disabled

    UInt32 layer = GetCollisionLayer(filterInfo);
    if (Config::options.ignoreLayers.count(layer)) return;

    {
        hkAabb aabb; phantom->calcAabb(aabb);
        aabb.expandBy(drawDistance);
        if (!aabb.containsPoint(NiPointToHkVector((*g_thePlayer)->pos * *g_havokWorldScale))) return;
    }

    NiColorA color = Config::options.phantomColor;

    if (phantom->getType() == hkpPhantomType::HK_PHANTOM_AABB) {
        hkpAabbPhantom *aabbPhantom = DYNAMIC_CAST(phantom, hkpPhantom, hkpAabbPhantom);
        if (!aabbPhantom) return;

        ShapeIdentifier shapeIdentifier = { wrapper, phantom, nullptr, nullptr };
        DrawAabb(shapeIdentifier, aabbPhantom->getAabb(), color);
    }
    else if (hkpShapePhantom *shapePhantom = DYNAMIC_CAST(phantom, hkpPhantom, hkpShapePhantom)) {
        const hkpShape *shape = phantom->getCollidable()->getShape();
        if (!shape) return;

        bhkShape *shapeWrapper = (bhkShape *)shape->m_userData;
        if (!shapeWrapper) return;

        const hkTransform &transform = shapePhantom->getTransform();

        ShapeIdentifier shapeIdentifier = { wrapper, phantom, shapeWrapper, shape };
        DrawShape(shapeIdentifier, shape, hkTransformToNiTransform(transform), color);
    }
}

void DrawIsland(const hkpSimulationIsland *island, float drawDistance)
{
    for (hkpEntity *entity : island->getEntities()) {
        if (hkpRigidBody *rigidBody = DYNAMIC_CAST(entity, hkpEntity, hkpRigidBody)) {
            DrawRigidBody(rigidBody, drawDistance);
        }
    }
}

bhkWorld *g_prevWorld = nullptr;

void DrawCollision()
{
    PlayerCharacter *player = *g_thePlayer;
    if (!player->GetNiNode()) return;

    TESObjectCELL *cell = player->parentCell;
    if (!cell) return;

    bool isInterior = cell->unk040 & 1;
    float drawDistance = isInterior ? Config::options.interiorDrawDistance : Config::options.exteriorDrawDistance;

    NiPointer<bhkWorld> world = GetHavokWorldFromCell(cell);
    if (!world) return;

    if (world != g_prevWorld) {
        // TODO: This doesn't cover if we travel between tamriel exterior and the whiterun city, or fast travelling
        // TODO: We should also remove entries that have not been used for a while, e.g. if we are outside which is all the same world
        g_shapeBuffers.clear();
    }
    g_prevWorld = world;

    g_renderGlobals->deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    g_renderGlobals->deviceContext->RSSetState(g_rasterizerState);

    g_renderGlobals->deviceContext->IASetInputLayout(g_inputLayout);

    g_renderGlobals->deviceContext->VSSetShader(g_vertexShader, nullptr, 0);
    g_renderGlobals->deviceContext->PSSetShader(g_pixelShader, nullptr, 0);

    { // Camera data (eye transforms)
        PerFrameVSData cameraData;
        cameraData.matProjView[0] = g_rendererShadowState->m_CameraData[0].m_ViewProjMat;
        cameraData.matProjView[1] = g_rendererShadowState->m_CameraData[1].m_ViewProjMat;

        D3D11_MAPPED_SUBRESOURCE mappedResource;
        g_renderGlobals->deviceContext->Map(g_cameraBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        memcpy(mappedResource.pData, &cameraData, sizeof(PerFrameVSData));
        g_renderGlobals->deviceContext->Unmap(g_cameraBuffer, 0);

        SetVSConstantBuffer(0, g_cameraBuffer);
    }

    {
        BSReadLocker lock(&world->worldLock);

        if (Config::options.drawActiveIslands) {
            for (const hkpSimulationIsland *island : world->world->m_activeSimulationIslands) {
                DrawIsland(island, drawDistance);
            }
        }

        if (Config::options.drawInactiveIslands) {
            for (const hkpSimulationIsland *island : world->world->m_inactiveSimulationIslands) {
                DrawIsland(island, drawDistance);
            }
        }

        if (Config::options.drawFixedIsland) {
            DrawIsland(world->world->m_fixedIsland, drawDistance);
        }

        if (Config::options.drawPhantoms) {
            for (hkpPhantom *phantom : world->world->getPhantoms()) {
                DrawPhantom(phantom, drawDistance);
            }
        }
    }
}

bool g_drawCollisionInitialized = false;
bool g_wasRefractionDebugLastFrame = false;

auto DoColorPass_NiCamera_FinishAccumulatingPostResolveDepth_HookLoc = RelocPtr<_GetNodeFromCollidable>(0x1323E3E);
_NiCamera_FinishAccumulatingPostResolveDepth DoColorPass_NiCamera_FinishAccumulatingPostResolveDepth_Original = 0;

void DoColorPass_NiCamera_FinishAccumulatingPostResolveDepth_Hook(NiCamera *camera, void *shaderAccumulator, UInt32 flags)
{
    bool refractionDebug = *g_refractionDebug;
    if (refractionDebug) {
        // Use ToggleRefractionDebug (unused console command) to toggle our collision visualizer

        if (!g_wasRefractionDebugLastFrame) {
            _MESSAGE("%d: Draw collision on", *g_currentFrameCounter);
            Config::ReloadIfModified();
        }

        if (!g_drawCollisionInitialized) {
            CreateShaders();
            CreateConstantBuffers();
            CreateRasterizerState();

            auto [vertices, indices] = GetSphereVertices(Config::options.constraintPivotSphereRadius);
            g_pivotSphereBuffers = CreateVertexAndIndexBuffers(vertices, indices);

            g_drawCollisionInitialized = true;
        }

        DrawCollision();
    }
    else if (g_wasRefractionDebugLastFrame) {
        _MESSAGE("%d: Draw collision off", *g_currentFrameCounter);

        if (Config::options.resetOnToggle) {
            g_shapeBuffers.clear();
        }
    }

    g_wasRefractionDebugLastFrame = refractionDebug;

    DoColorPass_NiCamera_FinishAccumulatingPostResolveDepth_Original(camera, shaderAccumulator, flags);
}


std::uintptr_t Write5Call(std::uintptr_t a_src, std::uintptr_t a_dst)
{
    const auto disp = reinterpret_cast<std::int32_t *>(a_src + 1);
    const auto nextOp = a_src + 5;
    const auto func = nextOp + *disp;

    g_branchTrampoline.Write5Call(a_src, a_dst);

    return func;
}


void PerformHooks()
{
    {
        std::uintptr_t originalFunc = Write5Call(DoColorPass_NiCamera_FinishAccumulatingPostResolveDepth_HookLoc.GetUIntPtr(), uintptr_t(DoColorPass_NiCamera_FinishAccumulatingPostResolveDepth_Hook));
        DoColorPass_NiCamera_FinishAccumulatingPostResolveDepth_Original = (_NiCamera_FinishAccumulatingPostResolveDepth)originalFunc;
        _MESSAGE("DoColorPass NiCamera::FinishAccumulatingPostResolveDepth hook complete");
    }
}

bool TryHook()
{
    // This should be sized to the actual amount used by your trampoline
    static const size_t TRAMPOLINE_SIZE = 32;

    if (g_trampoline) {
        void *branch = g_trampoline->AllocateFromBranchPool(g_pluginHandle, TRAMPOLINE_SIZE);
        if (!branch) {
            _ERROR("couldn't acquire branch trampoline from SKSE. this is fatal. skipping remainder of init process.");
            return false;
        }

        g_branchTrampoline.SetBase(TRAMPOLINE_SIZE, branch);

        void *local = g_trampoline->AllocateFromLocalPool(g_pluginHandle, TRAMPOLINE_SIZE);
        if (!local) {
            _ERROR("couldn't acquire codegen buffer from SKSE. this is fatal. skipping remainder of init process.");
            return false;
        }

        g_localTrampoline.SetBase(TRAMPOLINE_SIZE, local);
    }
    else {
        if (!g_branchTrampoline.Create(TRAMPOLINE_SIZE)) {
            _ERROR("couldn't create branch trampoline. this is fatal. skipping remainder of init process.");
            return false;
        }
        if (!g_localTrampoline.Create(TRAMPOLINE_SIZE, nullptr))
        {
            _ERROR("couldn't create codegen buffer. this is fatal. skipping remainder of init process.");
            return false;
        }
    }

    PerformHooks();
    return true;
}

extern "C" {
    void OnDataLoaded()
    {
    }

    void OnInputLoaded()
    {
    }

    // Listener for SKSE Messages
    void OnSKSEMessage(SKSEMessagingInterface::Message* msg)
    {
        if (msg) {
            if (msg->type == SKSEMessagingInterface::kMessage_InputLoaded) {
                OnInputLoaded();
            }
            else if (msg->type == SKSEMessagingInterface::kMessage_DataLoaded) {
                OnDataLoaded();
            }
        }
    }

    bool SKSEPlugin_Query(const SKSEInterface* skse, PluginInfo* info)
    {
        gLog.OpenRelative(CSIDL_MYDOCUMENTS, "\\My Games\\Skyrim VR\\SKSE\\collviz_vr.log");
        gLog.SetPrintLevel(IDebugLog::kLevel_DebugMessage);
        gLog.SetLogLevel(IDebugLog::kLevel_DebugMessage);

        _MESSAGE("Collision Visualizer VR v%s", COLLVIZ_VERSION_VERSTRING);

        info->infoVersion = PluginInfo::kInfoVersion;
        info->name = "Collision Visualizer VR";
        info->version = COLLVIZ_VERSION_MAJOR;

        g_pluginHandle = skse->GetPluginHandle();

        if (skse->isEditor) {
            _FATALERROR("[FATAL ERROR] Loaded in editor, marking as incompatible!\n");
            return false;
        }
        else if (skse->runtimeVersion != RUNTIME_VR_VERSION_1_4_15) {
            _FATALERROR("[FATAL ERROR] Unsupported runtime version %08X!\n", skse->runtimeVersion);
            return false;
        }

        return true;
    }

    bool SKSEPlugin_Load(const SKSEInterface * skse)
    {	// Called by SKSE to load this plugin
        _MESSAGE("Collision Visualizer VR loaded");

        if (Config::ReadConfigOptions()) {
            _MESSAGE("Successfully read config parameters");
        }
        else {
            _WARNING("[WARNING] Failed to read config options. Using defaults instead.");
        }

        gLog.SetPrintLevel((IDebugLog::LogLevel)Config::options.logLevel);
        gLog.SetLogLevel((IDebugLog::LogLevel)Config::options.logLevel);

        _MESSAGE("Registering for SKSE messages");
        g_messaging = (SKSEMessagingInterface*)skse->QueryInterface(kInterface_Messaging);
        g_messaging->RegisterListener(g_pluginHandle, "SKSE", OnSKSEMessage);

        g_trampoline = (SKSETrampolineInterface *)skse->QueryInterface(kInterface_Trampoline);
        if (!g_trampoline) {
            _WARNING("Couldn't get trampoline interface");
        }
        if (!TryHook()) {
            _ERROR("[CRITICAL] Failed to perform hooks");
            return false;
        }

        return true;
    }
};
